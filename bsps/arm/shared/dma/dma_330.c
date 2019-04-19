/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (C) 2019 On-Line Applications Research Corporation (OAR)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bsp.h>
#include <bsp/dma.h>
#include <bsp/alt_dma.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/* TODO(kmoore) Does not currently support multiple DMA engines */
rtems_status_code dma_init(DMA_Config_t *dma_cfg, volatile uint32_t *base)
{
  rtems_status_code status = alt_dma_init(dma_cfg);
  if (status != RTEMS_SUCCESSFUL) {
    return status;
  }
  status = alt_dma_event_int_select(ALT_DMA_EVENT_ABORT, ALT_DMA_EVENT_SELECT_SIG_IRQ);
  if (status != RTEMS_SUCCESSFUL) {
    return status;
  }
  for (int i = 0; i < BSP_DMA_MAX_CHANNELS; i++) {
    status = alt_dma_event_int_select(i, ALT_DMA_EVENT_SELECT_SIG_IRQ);
    if (status != RTEMS_SUCCESSFUL) {
      return status;
    }
  }
  return status;
}

rtems_status_code dma_uninit(DMA_Config_t *dma_cfg)
{
  return alt_dma_uninit();
}

rtems_status_code dma_channel_alloc(DMA_Config_t *dma_cfg, DMA_Channel_t *channel, uint8_t channel_number)
{
  channel->channel = channel_number;
  return alt_dma_channel_alloc(channel_number);
}

rtems_status_code dma_channel_alloc_any(DMA_Config_t *dma_cfg, DMA_Channel_t *channel)
{
  return alt_dma_channel_alloc_any((ALT_DMA_CHANNEL_t *)&(channel->channel));
}

rtems_status_code dma_channel_free(DMA_Channel_t *channel)
{
  return alt_dma_channel_free(channel->channel);
}

rtems_status_code dma_channel_reset(DMA_Channel_t *channel)
{
  return alt_dma_channel_kill(channel->channel);
}

rtems_status_code dma_channel_is_stopped(DMA_Channel_t *channel,
                                                       bool *stopped)
{
  ALT_DMA_CHANNEL_STATE_t state;
  rtems_status_code ret = alt_dma_channel_state_get(channel->channel, &state);
  *stopped = false;
  if (state == ALT_DMA_CHANNEL_STATE_STOPPED) {
    *stopped = true;
  }
  return ret;
}

rtems_status_code dma_channel_is_faulted(DMA_Channel_t *channel,
                                                       bool *faulted)
{
  ALT_DMA_CHANNEL_STATE_t state;
  rtems_status_code ret = alt_dma_channel_state_get(channel->channel, &state);
  *faulted = false;
  if (state == ALT_DMA_CHANNEL_STATE_FAULTING) {
    *faulted = true;
  }
  return ret;
}

rtems_status_code dma_channel_int_status_get(DMA_Channel_t *channel, bool *channel_interrupt_high)
{
  rtems_status_code status = alt_dma_int_status_get(channel->channel);
  if (status <= 1) {
	  *channel_interrupt_high = status;
	  return RTEMS_SUCCESSFUL;
  }
  return status;
}

rtems_status_code dma_abort_int_status_get(bool *abort_interrupt_high)
{
  rtems_status_code status = alt_dma_int_status_get(ALT_DMA_EVENT_ABORT);
  if (status == 1 || status == 0) {
	  *abort_interrupt_high = status;
	  return RTEMS_SUCCESSFUL;
  }
  return status;
}

rtems_status_code dma_channel_int_clear(DMA_Channel_t *channel)
{
  return alt_dma_int_clear(channel->channel);
}

rtems_status_code dma_abort_int_clear(void)
{
  return alt_dma_int_clear(ALT_DMA_EVENT_ABORT);
}

rtems_status_code dma_copy_memory_to_memory(DMA_Channel_t *channel,
                                                        void *dest,
                                                        const void *src,
                                                        size_t size)
{
  return alt_dma_memory_to_memory(channel->channel,
                                  &(channel->dma_channel_pvt),
                                  dest, src, size,
                                  true,
                                  channel->channel);
}

rtems_status_code dma_copy_zero_to_memory(DMA_Channel_t *channel,
                                                        void *buf,
                                                        size_t size)
{
  return alt_dma_zero_to_memory(channel->channel,
                                &(channel->dma_channel_pvt),
                                buf, size,
                                true,
                                channel->channel);
}

rtems_status_code dma_copy_memory_to_register(DMA_Channel_t *channel,
                                           void *dst_reg,
                                           const void *src_buf,
                                           size_t count,
                                           uint32_t register_width_bits)
{
  return alt_dma_memory_to_register(channel->channel, &(channel->dma_channel_pvt),
                                    dst_reg, src_buf,
                                    count, register_width_bits,
                                    true, channel->channel);
}

rtems_status_code dma_copy_register_to_memory(DMA_Channel_t *channel,
                                           void *dst_buf,
                                           const void *src_reg,
                                           size_t count,
                                           uint32_t register_width_bits)
{
  return alt_dma_register_to_memory(channel->channel, &(channel->dma_channel_pvt),
                                    dst_buf, src_reg,
                                    count, register_width_bits,
                                    true, channel->channel);
}

#ifdef __cplusplus
}
#endif  /* __cplusplus */
