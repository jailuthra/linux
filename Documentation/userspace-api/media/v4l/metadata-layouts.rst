.. SPDX-License-Identifier: GPL-2.0 OR GFDL-1.1-no-invariants-or-later

.. _media_metadata_layouts:

Metadata Layouts
----------------

The :ref:`metadata layout control <image_source_control_metadata_layout>`
specifies the exact layout of the metadata stream while the a :ref:`generic
metadata mbus code <media-bus-format-generic-meta>` on the subdevice pads
only describe the size of the :term:`Data Unit`.

.. _media-metadata-layout-ccs:

MIPI CCS Embedded Data Layout (``V4L2_METADATA_LAYOUT_CCS``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`MIPI CCS <https://www.mipi.org/specifications/camera-command-set>`_ defines a
metadata layout for sensor embedded data, identified by
``V4L2_CID_METADATA_LAYOUT`` control value ``V4L2_METADATA_LAYOUT_CCS``, which
is used to store the register configuration used for capturing a given
frame. The layout itself is defined in the CCS specification.

The CCS embedded data format (code ``0xa``) definition includes three levels:

1. Padding within CSI-2 bus :term:`Data Unit` as documented in the MIPI CCS
   specification.

2. The tagged data format as documented in the MIPI CCS specification.

3. Register addresses and register documentation as documented in the MIPI CCS
   specification.

The ``V4L2_METADATA_LAYOUT_CCS`` metadata layout value shall be used only by
devices that fulfill all three levels above.

This metadata layout code is only used for "2-byte simplified tagged data
format" (code ``0xa``) but their use may be extended further in the future, to
cover other CCS embedded data format codes.

Also see :ref:`CCS driver documentation <media-ccs-routes>`.

.. _media-metadata-layout-imx678:

Sony IMX678 Embedded Data Layout (``V4L2_METADATA_LAYOUT_IMX678``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Sony IMX678 camera sensor produces the following embedded data layout,
indicated by ``V4L2_METADATA_LAYOUT_IMX678`` metadata layout. The format
conforms to :ref:`CCS embedded data layout <media-metadata-layout-ccs>` up to
level 1.

Undocumented offsets till 170 may be ignored. From 171 to 288 all bytes are 00h
and from 289 to line length all bytes are 07h.

.. flat-table:: Sony IMX678 Embedded Data Layout. Octets at indices marked
                ignored have been omitted from the table. Values for multi-byte
                registers are in little-endian byte order.
    :header-rows: 1

    * - Offset
      - Size in bits (active bits if not the same as size)
      - Content description
    * - 2
      - 8 (6--0)
      - CFMODE (6--5) | WINMODE (3--0)
    * - 3
      - 8 (5)
      - HREVERSE
    * - 9
      - 8 (5)
      - VREVERSE
    * - 11
      - 8 (7--6)
      - ADBIT
    * - 13
      - 8 (3--0)
      - MDBIT (3) | LANEMODE (2--0)
    * - 24
      - 24 (20--0)
      - SHR0
    * - 54
      - 16 (12--0)
      - BLKLEVEL
