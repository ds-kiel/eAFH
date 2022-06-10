.. _vulnerabilities:

Vulnerabilities
###############

This page collects all of the vulnerabilities that are discovered and
fixed in each release.  It will also often have more details than is
available in the releases.  Some vulnerabilities are deemed to be
sensitive, and will not be publically discussed until there is
sufficient time to fix them.  Because the release notes are locked to
a version, the information here can be updated after the embargo is
lifted.

CVE-2017
========

CVE-2017-14199
--------------

Buffer overflow in :code:`getaddrinfo()`.

- `CVE-2017-14199 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2017-14199>`_

- `Zephyr project bug tracker ZEPSEC-12
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-12>`_

- `PR6158 fix for 1.11.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/6158>`_

CVE-2017-14201
--------------

The shell DNS command can cause unpredictable results due to misuse of
stack variables.

Use After Free vulnerability in the Zephyr shell allows a serial or
telnet connected user to cause denial of service, and possibly remote
code execution.

This has been fixed in release v1.14.0.

- `CVE-2017-14201 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2017-14201>`_

- `Zephyr project bug tracker ZEPSEC-17
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-17>`_

- `PR13260 fix for v1.14.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/13260>`_

CVE-2017-14202
--------------

The shell implementation does not protect against buffer overruns
resulting in unpredicable behavior.

Improper Restriction of Operations within the Bounds of a Memory
Buffer vulnerability in the shell component of Zephyr allows a serial
or telnet connected user to cause a crash, possibly with arbitrary
code execution.

This has been fixed in release v1.14.0.

- `CVE-2017-14202 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2017-14202>`_

- `Zephyr project bug tracker ZEPSEC-18
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-18>`_

- `PR13048 fix for v1.14.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/13048>`_

CVE-2019
========

CVE-2019-9506
-------------

The Bluetooth BR/EDR specification up to and including version 5.1
permits sufficiently low encryption key length and does not prevent an
attacker from influencing the key length negotiation. This allows
practical brute-force attacks (aka "KNOB") that can decrypt traffic
and inject arbitrary ciphertext without the victim noticing.

- `CVE-2019-9506 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2019-9506>`_

- `Zephyr project bug tracker ZEPSEC-20
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-20>`_

- `PR18702 fix for v1.14.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/18702>`_

- `PR18659 fix for v2.0.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/18659>`_

CVE-2020
========

CVE-2020-10019
--------------

Buffer Overflow vulnerability in USB DFU of zephyr allows a USB
connected host to cause possible remote code execution.

This has been fixed in releases v1.14.2, v2.2.0, and v2.1.1.

- `CVE-2020-10019 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10019>`_

- `Zephyr project bug tracker ZEPSEC-25
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-25>`_

- `PR23460 fix for 1.14.x
  <https://github.com/zephyrproject-rtos/zephyr/pull/23460>`_

- `PR23457 fix for 2.1.x
  <https://github.com/zephyrproject-rtos/zephyr/pull/23457>`_

- `PR23190 fix in 2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23190>`_

CVE-2020-10021
--------------

Out-of-bounds write in USB Mass Storage with unaligned sizes

Out-of-bounds Write in the USB Mass Storage memoryWrite handler with
unaligned Sizes.

See NCC-ZEP-024, NCC-ZEP-025, NCC-ZEP-026

This has been fixed in releases v1.14.2, and v2.2.0.

- `CVE-2020-10021 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10021>`_

- `Zephyr project bug tracker ZEPSEC-26
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-26>`_

- `PR23455 fix for v1.14.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/23455>`_

- `PR23456 fix for the v2.1 branch
  <https://github.com/zephyrproject-rtos/zephyr/pull/23456>`_

- `PR23240 fix for v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23240>`_

CVE-2020-10022
--------------

UpdateHub Module Copies a Variable-Size Hash String Into a Fixed-Size Array

A malformed JSON payload that is received from an UpdateHub server may
trigger memory corruption in the Zephyr OS. This could result in a
denial of service in the best case, or code execution in the worst
case.

See NCC-ZEP-016

This has been fixed in the below pull requests for master, branch from
v2.1.0, and branch from v2.2.0.

- `CVE-2020-10022 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10022>`_

- `Zephyr project bug tracker ZEPSEC-28
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-28>`_

- `PR24154 fix for master
  <https://github.com/zephyrproject-rtos/zephyr/pull/24154>`_

- `PR24065 fix for branch from v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/24065>`_

- `PR24066 fix for branch from v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/24066>`_

CVE-2020-10023
--------------

Shell Subsystem Contains a Buffer Overflow Vulnerability In
shell_spaces_trim

The shell subsystem contains a buffer overflow, whereby an adversary
with physical access to the device is able to cause a memory
corruption, resulting in denial of service or possibly code execution
within the Zephyr kernel.

See NCC-ZEP-019

This has been fixed in releases v1.14.2, v2.2.0, and in a branch from
v2.1.0,

- `CVE-2020-10023 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10023>`_

- `Zephyr project bug tracker ZEPSEC-29
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-29>`_

- `PR23646 fix for v1.14.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/23646>`_

- `PR23649 fix for branch from v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23649>`_

- `PR23304 fix for v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23304>`_

CVE-2020-10024
--------------

ARM Platform Uses Signed Integer Comparison When Validating Syscall
Numbers

The arm platform-specific code uses a signed integer comparison when
validating system call numbers. An attacker who has obtained code
execution within a user thread is able to elevate privileges to that
of the kernel.

See NCC-ZEP-001

This has been fixed in releases v1.14.2, and v2.2.0, and in a branch
from v2.1.0,

- `CVE-2020-10024 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10024>`_

- `Zephyr project bug tracker ZEPSEC-30
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-30>`_

- `PR23535 fix for v1.14.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/23535>`_

- `PR23498 fix for branch from v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23498>`_

- `PR23323 fix for v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23323>`_

CVE-2020-10027
--------------

ARC Platform Uses Signed Integer Comparison When Validating Syscall
Numbers

An attacker who has obtained code execution within a user thread is
able to elevate privileges to that of the kernel.

See NCC-ZEP-001

This has been fixed in releases v1.14.2, and v2.2.0, and in a branch
from v2.1.0.

- `CVE-2020-10027 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10027>`_

- `Zephyr project bug tracker ZEPSEC-35
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-35>`_

- `PR23500 fix for v1.14.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/23500>`_

- `PR23499 fix for branch from v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23499>`_

- `PR23328 fix for v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23328>`_

CVE-2020-10028
--------------

Multiple Syscalls In GPIO Subsystem Performs No Argument Validation

Multiple syscalls with insufficient argument validation

See NCC-ZEP-006

This has been fixed in releases v1.14.2, and v2.2.0, and in a branch
from v2.1.0.

- `CVE-2020-10028 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10028>`_

- `Zephyr project bug tracker ZEPSEC-32
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-32>`_

- `PR23733 fix for v1.14.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/23733>`_

- `PR23737 fix for branch from v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23737>`_

- `PR23308 fix for v2.2.0 (gpio patch)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23308>`_

CVE-2020-10058
--------------

Multiple Syscalls In kscan Subsystem Performs No Argument Validation

Multiple syscalls in the Kscan subsystem perform insufficient argument
validation, allowing code executing in userspace to potentially gain
elevated privileges.

See NCC-ZEP-006

This has been fixed in a branch from v2.1.0, and release v2.2.0.

- `CVE-2020-10058 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10058>`_

- `Zephyr project bug tracker ZEPSEC-34
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-34>`_

- `PR23748 fix for branch from v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23748>`_

- `PR23308 fix for v2.2.0 (kscan patch)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23308>`_

CVE-2020-10059
--------------

UpdateHub Module Explicitly Disables TLS Verification

The UpdateHub module disables DTLS peer checking, which allows for a
man in the middle attack. This is mitigated by firmware images
requiring valid signatures. However, there is no benefit to using DTLS
without the peer checking.

See NCC-ZEP-018

This has been fixed in a PR against Zephyr master.

- `CVE-2020-10059 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10059>`_

- `Zephyr project bug tracker ZEPSEC-36
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-36>`_

- `PR24954 fix on master (to be fixed in v2.3.0)
  <https://github.com/zephyrproject-rtos/zephyr/pull/24954>`_

- `PR24954 fix v2.1.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/24999>`_

- `PR24954 fix v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/24997>`_

CVE-2020-10060
--------------

UpdateHub Might Dereference An Uninitialized Pointer

In updatehub_probe, right after JSON parsing is complete, objects\[1]
is accessed from the output structure in two different places. If the
JSON contained less than two elements, this access would reference
unitialized stack memory. This could result in a crash, denial of
service, or possibly an information leak.

Recommend disabling updatehub until such a time as a fix can be made
available.

See NCC-ZEP-030

This issue has not been fixed.

- `CVE-2020-10060 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10060>`_

- `Zephyr project bug tracker ZEPSEC-37
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-37>`_

CVE-2020-10061
--------------

Error handling invalid packet sequence

Improper handling of the full-buffer case in the Zephyr Bluetooth
implementation can result in memory corruption.

This has been fixed in branches for v1.14.0, v2.2.0, and will be
included in v2.3.0.

- `CVE-2020-10061 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10061>`_

- `Zephyr project bug tracker ZEPSEC-75
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-75>`_

- `PR23516 fix for v2.3 (split driver)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23516>`_

- `PR23517 fix for v2.3 (legacy driver)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23517>`_

- `PR23091 fix for branch from v1.14.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23091>`_

- `PR23547 fix for branch from v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23547>`_

CVE-2020-10062
--------------

Packet length decoding error in MQTT

CVE: An off-by-one error in the Zephyr project MQTT packet length
decoder can result in memory corruption and possible remote code
execution. NCC-ZEP-031

The MQTT packet header length can be 1 to 4 bytes. An off-by-one error
in the code can result in this being interpreted as 5 bytes, which can
cause an integer overflow, resulting in memory corruption.

This has been fixed in master for v2.3.

- `CVE-2020-10062 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10062>`_

- `Zephyr project bug tracker ZEPSEC-84
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-84>`_

- `commit 11b7a37d for v2.3
  <https://github.com/zephyrproject-rtos/zephyr/pull/23821/commits/11b7a37d9a0b438270421b224221d91929843de4>`_

- `NCC-ZEP report`_ (NCC-ZEP-031)

.. _NCC-ZEP report: https://research.nccgroup.com/2020/05/26/research-report-zephyr-and-mcuboot-security-assessment

CVE-2020-10063
--------------

Remote Denial of Service in CoAP Option Parsing Due To Integer
Overflow

A remote adversary with the ability to send arbitrary CoAP packets to
be parsed by Zephyr is able to cause a denial of service.

This has been fixed in master for v2.3.

- `CVE-2020-10063 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10063>`_

- `Zephyr project bug tracker ZEPSEC-55
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-55>`_

- `PR24435 fix in master for v2.3
  <https://github.com/zephyrproject-rtos/zephyr/pull/24435>`_

- `PR24531 fix for branch from v2.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/24531>`_

- `PR24535 fix for branch from v2.1
  <https://github.com/zephyrproject-rtos/zephyr/pull/24535>`_

- `PR24530 fix for branch from v1.14
  <https://github.com/zephyrproject-rtos/zephyr/pull/24530>`_

- `NCC-ZEP report`_ (NCC-ZEP-032)

CVE-2020-10067
--------------

Integer Overflow In is_in_region Allows User Thread To Access Kernel Memory

A malicious userspace application can cause a integer overflow and
bypass security checks performed by system call handlers. The impact
would depend on the underlying system call and can range from denial
of service to information leak to memory corruption resulting in code
execution within the kernel.

See NCC-ZEP-005

This has been fixed in releases v1.14.2, and v2.2.0.

- `CVE-2020-10067 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10067>`_

- `Zephyr project bug tracker ZEPSEC-27
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-27>`_

- `PR23653 fix for v1.14.2
  <https://github.com/zephyrproject-rtos/zephyr/pull/23653>`_

- `PR23654 fix for the v2.1 branch
  <https://github.com/zephyrproject-rtos/zephyr/pull/23654>`_

- `PR23239 fix for v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23239>`_

CVE-2020-10068
--------------

Zephyr Bluetooth DLE duplicate requests vulnerability

In the Zephyr project Bluetooth subsystem, certain duplicate and
back-to-back packets can cause incorrect behavior, resulting in a
denial of service.

This has been fixed in branches for v1.14.0, v2.2.0, and will be
included in v2.3.0.

- `CVE-2020-10068 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10068>`_

- `Zephyr project bug tracker ZEPSEC-78
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-78>`_

- `PR23707 fix for v2.3 (split driver)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23707>`_

- `PR23708 fix for v2.3 (legacy driver)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23708>`_

- `PR23091 fix for branch from v1.14.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23091>`_

- `PR23964 fix for v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23964>`_

CVE-2020-10069
--------------

Zephyr Bluetooth unchecked packet data results in denial of service

An unchecked parameter in bluetooth data can result in an assertion
failure, or division by zero, resulting in a denial of service attack.

This has been fixed in branches for v1.14.0, v2.2.0, and will be
included in v2.3.0.

- `CVE-2020-10069 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10069>`_

- `Zephyr project bug tracker ZEPSEC-81
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-81>`_

- `PR23705 fix for v2.3 (split driver)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23705>`_

- `PR23706 fix for v2.3 (legacy driver)
  <https://github.com/zephyrproject-rtos/zephyr/pull/23706>`_

- `PR23091 fix for branch from v1.14.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23091>`_

- `PR23963 fix for branch from v2.2.0
  <https://github.com/zephyrproject-rtos/zephyr/pull/23963>`_

CVE-2020-10070
--------------

MQTT buffer overflow on receive buffer

In the Zephyr Project MQTT code, improper bounds checking can result
in memory corruption and possibly remote code execution.  NCC-ZEP-031

When calculating the packet length, arithmetic overflow can result in
accepting a receive buffer larger than the available buffer space,
resulting in user data being written beyond this buffer.

This has been fixed in master for v2.3.

- `CVE-2020-10070 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10070>`_

- `Zephyr project bug tracker ZEPSEC-85
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-85>`_

- `commit 0b39cbf3 for v2.3
  <https://github.com/zephyrproject-rtos/zephyr/pull/23821/commits/0b39cbf3c01d7feec9d0dd7cc7e0e374b6113542>`_

- `NCC-ZEP report`_ (NCC-ZEP-031)

CVE-2020-10071
--------------

Insufficient publish message length validation in MQTT

The Zephyr MQTT parsing code performs insufficient checking of the
length field on publish messages, allowing a buffer overflow and
potentially remote code execution. NCC-ZEP-031

This has been fixed in master for v2.3.

- `CVE-2020-10071 <http://cve.mitre.org/cgi-bin/cvename.cgi?name=CVE-2020-10071>`_

- `Zephyr project bug tracker ZEPSEC-86
  <https://zephyrprojectsec.atlassian.net/browse/ZEPSEC-86>`_

- `commit 989c4713 fix for v2.3
  <https://github.com/zephyrproject-rtos/zephyr/pull/23821/commits/989c4713ba429aa5105fe476b4d629718f3e6082>`_

- `NCC-ZEP report`_ (NCC-ZEP-031)
