from m5.objects.SimpleMemory import SimpleMemory
from m5.params import *
from m5.proxy import *


class MessMemory(SimpleMemory):
    type = "MessMemory"
    cxx_header = "mem/mess_mem.hh"
    cxx_class = "gem5::memory::MessMemory"

    curves_path = Param.String(
        "",
        "Path to the directory containing the Mess bandwidth-latency curves",
    )
    sampling_window = Param.Unsigned(
        20000,
        "Sampling window for monitoring memory traffic, in memory accesses",
    )
