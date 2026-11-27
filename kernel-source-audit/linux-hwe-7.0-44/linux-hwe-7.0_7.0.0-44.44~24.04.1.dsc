-----BEGIN PGP SIGNED MESSAGE-----
Hash: SHA512

Format: 1.0
Source: linux-hwe-7.0
Binary: linux-hwe-7.0-headers-7.0.0-44, linux-hwe-7.0-tools-7.0.0-44, linux-hwe-7.0-cloud-tools-7.0.0-44, linux-image-unsigned-7.0.0-44-generic, linux-image-unsigned-7.0.0-44-generic-dbgsym, linux-image-7.0.0-44-generic, linux-image-7.0.0-44-generic-dbgsym, linux-modules-7.0.0-44-generic, linux-headers-7.0.0-44-generic, linux-lib-rust-7.0.0-44-generic, linux-tools-7.0.0-44-generic, linux-cloud-tools-7.0.0-44-generic, linux-buildinfo-7.0.0-44-generic, linux-image-unsigned-7.0.0-44-generic-64k, linux-image-unsigned-7.0.0-44-generic-64k-dbgsym, linux-modules-7.0.0-44-generic-64k, linux-headers-7.0.0-44-generic-64k, linux-lib-rust-7.0.0-44-generic-64k, linux-tools-7.0.0-44-generic-64k, linux-cloud-tools-7.0.0-44-generic-64k, linux-buildinfo-7.0.0-44-generic-64k
Architecture: all amd64 armhf arm64 ppc64el s390x
Version: 7.0.0-44.44~24.04.1
Maintainer: Ubuntu Kernel Team <kernel-team@lists.ubuntu.com>
Standards-Version: 3.9.4.0
Vcs-Git: git://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/noble -b hwe-7.0
Build-Depends: gcc-13, gcc-13-aarch64-linux-gnu [arm64] <cross>, gcc-13-arm-linux-gnueabihf [armhf] <cross>, gcc-13-powerpc64le-linux-gnu [ppc64el] <cross>, gcc-13-riscv64-linux-gnu [riscv64] <cross>, gcc-13-s390x-linux-gnu [s390x] <cross>, gcc-13-x86-64-linux-gnu [amd64] <cross>, autoconf <!stage1>, automake <!stage1>, bc <!stage1>, bindgen-0.65 [amd64 arm64 armhf ppc64el riscv64 s390x], bison <!stage1>, clang-19 [amd64 arm64 armhf ppc64el riscv64 s390x], cpio, curl <!stage1>, debhelper-compat (= 10), default-jdk-headless <!stage1>, dkms <!stage1>, flex <!stage1>, gawk <!stage1>, java-common <!stage1>, kmod <!stage1>, libaudit-dev <!stage1>, libbpf-dev <!stage1>, libcap-dev <!stage1>, libdebuginfod-dev [amd64 arm64 armhf ppc64el s390x riscv64] <!stage1>, libdw-dev <!stage1>, libelf-dev <!stage1>, libiberty-dev <!stage1>, liblzma-dev <!stage1>, libnewt-dev <!stage1>, libnl-3-dev, libnl-genl-3-dev, libnuma-dev [amd64 arm64 ppc64el s390x] <!stage1>, libpci-dev <!stage1>, libssl-dev <!stage1>, libstdc++-13-dev, libtool <!stage1>, libtraceevent-dev [amd64 arm64 armhf ppc64el s390x riscv64] <!stage1>, libtracefs-dev [amd64 arm64 armhf ppc64el s390x riscv64] <!stage1>, libudev-dev <!stage1>, libunwind8-dev [amd64 arm64 armhf ppc64el] <!stage1>, llvm-19-dev, makedumpfile [amd64] <!stage1>, openssl <!stage1>, pahole [amd64 arm64 armhf ppc64el s390x riscv64] | dwarves (>= 1.21) [amd64 arm64 armhf ppc64el s390x riscv64] <!stage1>, pkg-config <!stage1>, python3 <!stage1>, python3-dev <!stage1>, python3-setuptools, rsync [!i386] <!stage1>, rust-1.91-src [amd64 arm64 armhf ppc64el riscv64 s390x], rustc-1.91 [amd64 arm64 armhf ppc64el riscv64 s390x], rustfmt-1.91 [amd64 arm64 armhf ppc64el riscv64 s390x], uuid-dev <!stage1>, zstd <!stage1>
Build-Depends-Indep: asciidoc <!stage1>, bzip2 <!stage1>, python3-docutils <!stage1>, sharutils <!stage1>, xmlto <!stage1>
Package-List:
 linux-buildinfo-7.0.0-44-generic deb kernel optional arch=amd64,armhf,arm64,ppc64el,s390x profile=!stage1
 linux-buildinfo-7.0.0-44-generic-64k deb kernel optional arch=arm64 profile=!stage1
 linux-cloud-tools-7.0.0-44-generic deb devel optional arch=amd64,armhf,arm64,ppc64el,s390x profile=!stage1
 linux-cloud-tools-7.0.0-44-generic-64k deb devel optional arch=arm64 profile=!stage1
 linux-headers-7.0.0-44-generic deb devel optional arch=amd64,armhf,arm64,ppc64el,s390x profile=!stage1
 linux-headers-7.0.0-44-generic-64k deb devel optional arch=arm64 profile=!stage1
 linux-hwe-7.0-cloud-tools-7.0.0-44 deb devel optional arch=amd64,armhf profile=!stage1
 linux-hwe-7.0-headers-7.0.0-44 deb devel optional arch=all profile=!stage1
 linux-hwe-7.0-tools-7.0.0-44 deb devel optional arch=amd64,armhf,arm64,ppc64el,s390x profile=!stage1
 linux-image-7.0.0-44-generic deb kernel optional arch=armhf,ppc64el profile=!stage1
 linux-image-7.0.0-44-generic-dbgsym deb devel optional arch=armhf,ppc64el profile=!stage1
 linux-image-unsigned-7.0.0-44-generic deb kernel optional arch=amd64,arm64,s390x profile=!stage1
 linux-image-unsigned-7.0.0-44-generic-64k deb kernel optional arch=arm64 profile=!stage1
 linux-image-unsigned-7.0.0-44-generic-64k-dbgsym deb devel optional arch=arm64 profile=!stage1
 linux-image-unsigned-7.0.0-44-generic-dbgsym deb devel optional arch=amd64,arm64,s390x profile=!stage1
 linux-lib-rust-7.0.0-44-generic deb devel optional arch=amd64 profile=!stage1
 linux-lib-rust-7.0.0-44-generic-64k deb devel optional arch=amd64 profile=!stage1
 linux-modules-7.0.0-44-generic deb kernel optional arch=amd64,armhf,arm64,ppc64el,s390x profile=!stage1
 linux-modules-7.0.0-44-generic-64k deb kernel optional arch=arm64 profile=!stage1
 linux-tools-7.0.0-44-generic deb devel optional arch=amd64,armhf,arm64,ppc64el,s390x profile=!stage1
 linux-tools-7.0.0-44-generic-64k deb devel optional arch=arm64 profile=!stage1
Checksums-Sha1:
 84ccdabedcb5eb28af56989b6070cfa864a325a9 254937830 linux-hwe-7.0_7.0.0.orig.tar.gz
 fb850f7ef433567fa48f64aa85d0e4066bf0c6e3 6551438 linux-hwe-7.0_7.0.0-44.44~24.04.1.diff.gz
Checksums-Sha256:
 c6343795c8c7e4feff55f5b309365c8e0eaf49cd6abcb412e32315313f149536 254937830 linux-hwe-7.0_7.0.0.orig.tar.gz
 ca52b58456be235c683de4638689e1837a86ec6ba19c97c398fc26af06941501 6551438 linux-hwe-7.0_7.0.0-44.44~24.04.1.diff.gz
Files:
 415e5d4ee110d9415c2901751e6f9c9d 254937830 linux-hwe-7.0_7.0.0.orig.tar.gz
 9ae8ac655ef5bd984903037b4ad05e37 6551438 linux-hwe-7.0_7.0.0-44.44~24.04.1.diff.gz
Ubuntu-Compatible-Signing: ubuntu/4 pro/3 livepatch:2025 kmod:2025

-----BEGIN PGP SIGNATURE-----

iQJSBAEBCgA8FiEEYd3T8x9/RysNQgAIT1deSCm2VY0FAmrzcOIeHGFsaWNlLm11
bmR1cnVjYUBjYW5vbmljYWwuY29tAAoJEE9XXkgptlWN/IcP/ReOf/7tzHxrW5r3
zaxRORmEoQsC9zEo8/Fd7cALxq8HeWDvCVGhaY4CuR8rO2MowbyXVLkJnvWSqEGX
MO12gYMON9ekn10jVmDSzITdkV6Zq0vHAfh+w4VoB1nvEKh/iwKcXF3HJswMcnxb
4kp+adpta3yRe5/cqAg0I2r9f3Dm0pDB1GX6SZtGwVEYLn/Ct/SnfqlsMChkOWJC
eMuPBc5UVqVtm6kGbFxOmAag9gCH15xP81ZVAm8qiVeNW2JKqpKG/ixUqUvcivWb
Wa/NvZmXyxP3aho+aEv056zcMywWsbUAFNnfBXFikbX75JkFgX2opIzGbqU6Nc+p
kGww2sQvo47jTuAidjKyC76ClQkuvlbkhSCg5H6ortrO4EmHgQtTj6UuyNHXUG9x
BWF8LlvOO7Jd7S8c0SNndAU2iWGdeb3JmGimtinrw8HlmdFEQvJaXJ8vmO2y/mEJ
FZMHvBsWcJPtHq4mX6wohRunsSt+ekouxMmsrm+RjbeiCrEydgkeUBouCnKinTU+
2dhonsi7dTzWZqIN47+ijIYRchFyM6x0IUegP54uTI5Lvwv09i1lmaX8oc0XuAH+
KfNEWyCGrQcm2dv1lNwtTywMWpq6QPD1eaQ26IxSv+a8u7elFb4LEQlACg6cxcr7
MdYBKshNkmrUoD0DIPZ/lNJgcjx6
=EzBL
-----END PGP SIGNATURE-----
