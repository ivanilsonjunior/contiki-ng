/*
 * Copyright (c) 2015, SICS Swedish ICT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \file
 *         To test 6P transaction on a RPL+TSCH network
 *
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Shalu R <shalur@cdac.in>
 *         Lijo Thomas <lijo@cdac.in>
 */

#include "contiki.h"
#include "sys/node-id.h"
#include "sys/log.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/routing/routing.h"
#include "lib/random.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/simple-udp.h"
#include "net/routing/routing.h"
#include "net/routing/rpl-lite/rpl-dag.h"
#include "net/routing/rpl-lite/rpl-types.h"



#include "sf-simple.h"

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

#include <inttypes.h>
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO
#define UDP_PORT	8765

/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  uint32_t seqnum;
  memcpy(&seqnum, data, sizeof(seqnum));
  LOG_INFO("app receive packet seqnum=%" PRIu32 " from=", seqnum);
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "RPL Node");
PROCESS(my_app,"UDP APP");


AUTOSTART_PROCESSES(&node_process,&my_app);




/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
  static int is_coordinator;
  static struct etimer et;
  static struct tsch_neighbor *n;
  static struct tsch_neighbor *old_n;


  PROCESS_BEGIN();

  is_coordinator = 0;

#if CONTIKI_TARGET_COOJA
  is_coordinator = (node_id == 1);
#endif

  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }

  NETSTACK_MAC.on();
  sixtop_add_sf(&sf_simple_driver);

  etimer_set(&et, CLOCK_SECOND * 5);
  while(1) {
    PROCESS_YIELD_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    /* Get time-source neighbor */
    n = tsch_queue_get_time_source();

    if(!is_coordinator && n != NULL) {
      int diotime = curr_instance.dag.dio_intcurrent;
      int fila = tsch_queue_global_packet_count();
      if (diotime != 0) {
        struct tsch_slotframe *sf = tsch_schedule_slotframe_head();
        struct tsch_link *l = list_head(sf->links_list);
        int tsch_links = -1; // just for not count the Minimal "Catch-all" slot
        while(l != NULL) {
          tsch_links++;
          l = list_item_next(l);
        }
      /* Test if a parent switch happened*/
      if (old_n != n){
        for (size_t i = 1; i < tsch_links; i++)
        {
          sf_simple_remove_links(tsch_queue_get_nbr_address(old_n));
        }
      }

      int demanded_cell = ((20 - diotime) / 2) + (fila/20);
      if (tsch_links > demanded_cell){
        sf_simple_remove_links(tsch_queue_get_nbr_address(n));

      } else if (tsch_links < demanded_cell) {
        sf_simple_add_links(tsch_queue_get_nbr_address(n), 1);
      }
    }
    }
    old_n = n;
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(my_app, ev, data)
{
  static struct etimer periodic_timer;
  static uint32_t seqnum;
  static struct simple_udp_connection udp_conn;
  uip_ipaddr_t dest_address;
  static int is_coordinator;

  PROCESS_BEGIN();

  simple_udp_register(&udp_conn, UDP_PORT, NULL,
                      UDP_PORT, udp_rx_callback);

#if CONTIKI_TARGET_COOJA
  is_coordinator = (node_id == 1);
#endif

  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  } else {

    etimer_set(&periodic_timer, APP_WARM_UP_PERIOD_SEC * CLOCK_SECOND
        + random_rand() % (APP_SEND_INTERVAL_SEC * CLOCK_SECOND));

    while(1) {
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

      if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_address)) {
        /* generate a packet with a new seqnum */
        seqnum++;
        LOG_INFO("app generate packet seqnum=%" PRIu32 " node_id=%u\n", seqnum, node_id);
        simple_udp_sendto(&udp_conn, &seqnum, sizeof(seqnum), &dest_address);
      }

      etimer_set(&periodic_timer, APP_SEND_INTERVAL_SEC * CLOCK_SECOND);
    }
  }
  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
