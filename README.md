evg
===
Evg is an EPICS driver for controlling the VME-EVG230 event generator over the nework.

Background
==========
All control servers at SESAME run on Linux/x86 platforms. While the VME-EVG230 is typically controlled over the VME bus, there are some advantages in controlling it over the network instead:
* Drops the dead weight: The VME crate, the VME CPU card, the RTOS that runs on the CPU card (along with any required licenses), and the debug terminal that connects to the CPU card are no longer needed.
* Lowers the cost of implementation: A direct consequence of the point above.
* Confines the required development skills to Linux/x86 platforms. Knowledge in VME-bus, VxWorks, or any other RTOS/OS is not required.
* Maintains coherency in the control infrastructure (applies to SESAME since the rest of the IOC's at SESAME run on Linux/x86 platforms).

Features
========
The driver supports the following VME-EVG230 features:
* Enables/disables the VME-EVG230, sequencer, and AC trigger.
* Triggers the sequencer from AC mains.
* Programs the clock prescalers for RF, sequencer, AC trigger, and counters.
* Programs the event sequencer with timestamps and event codes.

The driver does not support the following features:
* Distributed bus and data transmission.
* Trigger events.
* Upstream events.
* The second event sequencer. Only the first one is supported.
* Triggering of the sequencer from TTL trigger inputs or multiplexed counters.
* Timestamping.

Installation
============
Clone the repository and integrate the driver with the EPICS/support framework. Look in the evg folder for examples of database and startup files.
