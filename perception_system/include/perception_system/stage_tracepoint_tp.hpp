#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER perception_system

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "perception_system/stage_tracepoint_tp.hpp"

#if !defined(PERCEPTION_SYSTEM__STAGE_TRACEPOINT_TP_HPP_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define PERCEPTION_SYSTEM__STAGE_TRACEPOINT_TP_HPP_

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(TRACEPOINT_PROVIDER,
                 image_proc_start,
                 TP_ARGS(uint64_t, frame_seq_arg, uint64_t, stamp_ns_arg),
                 TP_FIELDS(ctf_integer(uint64_t, frame_seq, frame_seq_arg)
                               ctf_integer(uint64_t, stamp_ns, stamp_ns_arg)))

TRACEPOINT_EVENT(TRACEPOINT_PROVIDER,
                 image_proc_end,
                 TP_ARGS(uint64_t, frame_seq_arg, uint64_t, stamp_ns_arg),
                 TP_FIELDS(ctf_integer(uint64_t, frame_seq, frame_seq_arg)
                               ctf_integer(uint64_t, stamp_ns, stamp_ns_arg)))

TRACEPOINT_EVENT(TRACEPOINT_PROVIDER,
                 detector_start,
                 TP_ARGS(uint64_t, frame_seq_arg, uint64_t, stamp_ns_arg),
                 TP_FIELDS(ctf_integer(uint64_t, frame_seq, frame_seq_arg)
                               ctf_integer(uint64_t, stamp_ns, stamp_ns_arg)))

TRACEPOINT_EVENT(TRACEPOINT_PROVIDER,
                 detector_end,
                 TP_ARGS(uint64_t, frame_seq_arg, uint64_t, stamp_ns_arg),
                 TP_FIELDS(ctf_integer(uint64_t, frame_seq, frame_seq_arg)
                               ctf_integer(uint64_t, stamp_ns, stamp_ns_arg)))

#endif

#include <lttng/tracepoint-event.h>
