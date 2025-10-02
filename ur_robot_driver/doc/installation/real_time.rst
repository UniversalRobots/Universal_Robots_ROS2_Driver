.. _real time setup:

Setup for real-time scheduling
==============================

In order to run the ``universal_robot_driver``, we highly recommend to setup a ubuntu system with
real-time capabilities. Especially with a robot from the e-Series the higher control frequency
might lead to non-smooth trajectory execution if not run using a real-time-enabled system.

You might still be able to control the robot using a non-real-time system. This is, however, not recommended.

While the best-performing strategy would be to use a real-time enabled kernel, using a lowlatency
kernel has shown to be sufficient in many situations which is why this is also shown as an option
here.

Installing a lowlatency-kernel
------------------------------

Installing a lowlatency kernel is pretty straightforward:

.. code-block:: console

   $ sudo apt install linux-lowlatency

Setting up Ubuntu with a PREEMPT_RT kernel
------------------------------------------

To get real-time support into a ubuntu system, the following steps have to be performed:

#. Get the sources of a real-time kernel
#. Compile the real-time kernel
#. Setup user privileges to execute real-time tasks

This guide will help you setup your system with a real-time kernel.

Preparing
^^^^^^^^^

To build the kernel, you will need a couple of tools available on your system. You can install them
using

.. code-block:: console

   $ sudo apt-get install build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison libelf-dev dwarves


.. note::

   For different kernel versions the dependencies might be different than that. If you experience
   problems such as ``fatal error: liXYZ.h: No such file or directory`` during compilation, try to
   install the library's corresponding ``dev``-package.

Before you download the sources of a real-time-enabled kernel, check the kernel version that is currently installed:

.. code-block:: console

   $ uname -r
   5.15.0-107-generic

To continue with this tutorial, please create a temporary folder and navigate into it. You should
have sufficient space (around 25GB) there, as the extracted kernel sources take much space. After
the new kernel is installed, you can delete this folder again.

In this example we will use a temporary folder inside our home folder:

.. code-block:: console

   $ mkdir -p ${HOME}/rt_kernel_build
   $ cd ${HOME}/rt_kernel_build

All future commands are expected to be run inside this folder. If the folder is different, the ``$``
sign will be prefixed with a path relative to the above folder.

Getting the sources for a real-time kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build a real-time kernel, we first need to get the kernel sources and the real-time patch.

First, we must decide on the kernel version that we want to use. Above, we
determined that our system has a 5.15 kernel installed. However, real-time
patches exist only for selected kernel versions. Those can be found on the
`linuxfoundation wiki <https://wiki.linuxfoundation.org/realtime/preempt_rt_versions>`_.

In this example, we will select a 5.15.158 kernel with RT patch version 76. Select a kernel version close  to the
one installed on your system. For easier reference later on we will export version information to
our shell environment. Make sure to execute all following commands in this shell.

.. code-block:: console

   $ export KERNEL_MAJOR_VERSION=5
   $ export KERNEL_MINOR_VERSION=15
   $ export KERNEL_PATCH_VERSION=158
   $ export RT_PATCH_VERSION=76
   $ export KERNEL_VERSION="$KERNEL_MAJOR_VERSION.$KERNEL_MINOR_VERSION.$KERNEL_PATCH_VERSION"

Go ahead and download the kernel sources, patch sources and their signature files:

.. code-block:: console

   $ wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/$KERNEL_MAJOR_VERSION.$KERNEL_MINOR_VERSION/patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.xz
   $ wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/$KERNEL_MAJOR_VERSION.$KERNEL_MINOR_VERSION/patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign
   $ wget https://www.kernel.org/pub/linux/kernel/v$KERNEL_MAJOR_VERSION.x/linux-$KERNEL_VERSION.tar.xz
   $ wget https://www.kernel.org/pub/linux/kernel/v$KERNEL_MAJOR_VERSION.x/linux-$KERNEL_VERSION.tar.sign

To unzip the downloaded files do

.. code-block:: console

   $ xz -dk patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.xz
   $ xz -d linux-$KERNEL_VERSION.tar.xz

Verification
~~~~~~~~~~~~

Technically, you can skip this section, it is however highly recommended to verify the file
integrity of such a core component of your system!

To verify file integrity, you must first import public keys by the kernel developers and the patch
author. For the kernel sources use (as suggested on
`kernel.org <https://www.kernel.org/signature.html>`_\ )

.. code-block:: console

   $ gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org

and for the patch view the gpg information

.. code-block:: console

   $ gpg2 --verify patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign
   gpg: assuming signed data in 'patch-5.15.158-rt76.patch'
   gpg: Signature made Fri May  3 17:12:45 2024 UTC
   gpg:                using RSA key AD85102A6BE1CDFE9BCA84F36CEF3D27CA5B141E
   gpg: Can't check signature: No public key

So, we need to import the key using

.. code-block:: console

   gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys AD85102A6BE1CDFE9BCA84F36CEF3D27CA5B141E


Now we can verify the downloaded sources:

.. code-block:: console

   $ gpg2 --verify linux-$KERNEL_VERSION.tar.sign
   gpg: assuming signed data in 'linux-5.15.158.tar'
   gpg: Signature made Thu May  2 14:28:07 2024 UTC
   gpg:                using RSA key 647F28654894E3BD457199BE38DBBDC86092693E
   gpg: Good signature from "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
   gpg: WARNING: This key is not certified with a trusted signature!
   gpg:          There is no indication that the signature belongs to the owner.
   Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E

and

.. code-block:: console

   $ gpg2 --verify patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign
   gpg: assuming signed data in 'patch-5.15.158-rt76.patch'
   gpg: Signature made Fri May  3 17:12:45 2024 UTC
   gpg:                using RSA key AD85102A6BE1CDFE9BCA84F36CEF3D27CA5B141E
   gpg: Good signature from "Joseph Salisbury <joseph.salisbury@canonical.com>" [unknown]
   gpg:                 aka "Joseph Salisbury <josephtsalisbury@gmail.com>" [unknown]
   gpg:                 aka "Joseph Salisbury <joseph.salisbury@ubuntu.com>" [unknown]
   gpg: WARNING: This key is not certified with a trusted signature!
   gpg:          There is no indication that the signature belongs to the owner.
   Primary key fingerprint: AD85 102A 6BE1 CDFE 9BCA  84F3 6CEF 3D27 CA5B 141E


Compilation
^^^^^^^^^^^

Before we can compile the sources, we have to extract the tar archive and apply the patch

.. code-block:: console

   $ tar xf linux-$KERNEL_VERSION.tar
   $ cd linux-$KERNEL_VERSION
   $ xzcat ../patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.xz | patch -p1

Now to configure your kernel, just type

.. code-block:: console

   $ make oldconfig

This will ask for kernel options. For everything else then the ``Preemption Model`` use the default
value (just press Enter) or adapt to your preferences. For the preemption model select ``Fully Preemptible Kernel``\ :

.. code-block:: console

   Preemption Model
     1. No Forced Preemption (Server) (PREEMPT_NONE)
   > 2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
     3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT)
     4. Fully Preemptible Kernel (Real-Time) (PREEMPT_RT) (NEW)
   choice[1-4?]: 4

On newer kernels you need to disable some key checking:

.. code-block:: console

   $ scripts/config --disable SYSTEM_TRUSTED_KEYS
   $ scripts/config --disable SYSTEM_REVOCATION_KEYS

Now you can build the kernel. This will take some time...

.. code-block:: console

   $ make -j $(getconf _NPROCESSORS_ONLN) deb-pkg

After building, install the ``linux-headers`` and ``linux-image`` packages in the parent folder (only
the ones without the ``-dbg`` in the name)

.. code-block:: console

   $ sudo apt install ../linux-headers-$KERNEL_VERSION-rt$RT_PATCH_VERSION*.deb \
                      ../linux-image-$KERNEL_VERSION-rt$RT_PATCH_VERSION*.deb

Setup user privileges to use real-time scheduling
-------------------------------------------------

To be able to schedule threads with user privileges (what the driver will do) you'll have to change
the user's limits by changing ``/etc/security/limits.conf`` (See `the manpage <https://manpages.ubuntu.com/manpages/jammy/en/man5/limits.conf.5.html>`_ for details)

We recommend to setup a group for real-time users instead of writing a fixed username into the config
file:

.. code-block:: console

   $ sudo groupadd realtime
   $ sudo usermod -aG realtime $(whoami)

Then, make sure ``/etc/security/limits.conf`` contains

.. code-block:: linuxconfig

   @realtime soft rtprio 99
   @realtime soft priority 99
   @realtime soft memlock 102400
   @realtime hard rtprio 99
   @realtime hard priority 99
   @realtime hard memlock 102400

Note: You will have to log out and log back in (Not only close your terminal window) for these
changes to take effect. No need to do this now, as we will reboot later on, anyway.

Setup GRUB to always boot the lowlatency / real-time kernel
-----------------------------------------------------------

To make the new kernel the default kernel that the system will boot into every time, you'll have to
change the grub config file inside ``/etc/default/grub``.

Note: This works for ubuntu, but might not be working for other linux systems. It might be necessary
to use another menuentry name there.

But first, let's find out the name of the entry that we will want to make the default. You can list
all available kernels using

.. code-block:: console

   $ awk -F\' '/menuentry |submenu / {print $1 $2}' /boot/grub/grub.cfg
   menuentry Ubuntu
   submenu Advanced options for Ubuntu
           menuentry Ubuntu, with Linux 5.15.158-rt76
           menuentry Ubuntu, with Linux 5.15.158-rt76 (recovery mode)
           menuentry Ubuntu, with Linux 5.15.0-107-lowlatency
           menuentry Ubuntu, with Linux 5.15.0-107-lowlatency (recovery mode)
           menuentry Ubuntu, with Linux 5.15.0-107-generic
           menuentry Ubuntu, with Linux 5.15.0-107-generic (recovery mode)

From the output above, we'll need to generate a string with the pattern ``"submenu_name>entry_name"``. In our case this would be

.. code-block:: text

   "Advanced options for Ubuntu>Ubuntu, with Linux 5.15.158-rt76"

**The double quotes and no spaces around the** ``>`` **are important!**

With this, we can setup the default grub entry and then update the grub menu entries. Don't forget this last step!

.. code-block:: console

   $ sudo sed -i "s/^GRUB_DEFAULT=.*/GRUB_DEFAULT=\"Advanced options for Ubuntu>Ubuntu, with Linux ${KERNEL_VERSION}-rt${RT_PATCH_VERSION}\"/" /etc/default/grub
   $ sudo update-grub

Reboot the PC
-------------

After having performed the above mentioned steps, reboot the PC. It should boot into the correct
kernel automatically.

Check for preemption capabilities
---------------------------------

Make sure that the kernel does indeed support real-time scheduling:

.. code-block:: console

   $ uname -v | cut -d" " -f1-4
   #1 SMP PREEMPT_RT Tue

Optional: Disable CPU speed scaling
-----------------------------------

Many modern CPUs support changing their clock frequency dynamically depending on the currently
requested computation resources. In some cases this can lead to small interruptions in execution.
While the real-time scheduled controller thread should be unaffected by this, any external
components such as a visual servoing system might be interrupted for a short period on scaling
changes.

To check and modify the power saving mode, install cpufrequtils:

.. code-block:: console

   $ sudo apt install cpufrequtils

Run ``cpufreq-info`` to check available "governors" and the current CPU Frequency (\ ``current CPU
frequency is XXX MHZ``\ ). In the following we will set the governor to "performance".

.. code-block:: console

   $ sudo systemctl disable ondemand
   $ sudo systemctl enable cpufrequtils
   $ sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
   $ sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils

This disables the ``ondemand`` CPU scaling daemon, creates a ``cpufrequtils`` config file and restarts
the ``cpufrequtils`` service. Check with ``cpufreq-info``.

For further information about governors, please see the `kernel
documentation <https://www.kernel.org/doc/Documentation/cpu-freq/governors.txt>`_.
