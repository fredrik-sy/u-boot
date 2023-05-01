cd ..
mkdir images
cd images
mkdir DB-78XX0-A-BP
mkdir DB-78200-A-BP
mkdir RD-78200-A-AMC
cd DB-78XX0-A-BP
mkdir LE
mkdir BE

cd ../DB-78200-A-BP
mkdir LE
mkdir BE
cd LE
mkdir CV
mkdir DUAL_LINUX
cd ../BE
mkdir CV
mkdir DUAL_LINUX

cd ../../RD-78200-A-AMC
mkdir LE

cd ../../u-boot-x.x.x

make mrproper
make db78XX0_config LE=1
make -s
cp u-boot-db78XX0.bin ../images/DB-78XX0-A-BP/LE/
cp u-boot-db78XX0.srec ../images/DB-78XX0-A-BP/LE/
cp u-boot-db78XX0 ../images/DB-78XX0-A-BP/LE/

make mrproper
make db78XX0_config BE=1
make -s
cp u-boot-db78XX0.bin ../images/DB-78XX0-A-BP/BE/
cp u-boot-db78XX0.srec ../images/DB-78XX0-A-BP/BE/
cp u-boot-db78XX0 ../images/DB-78XX0-A-BP/BE/

make mrproper
make db78200_MP_config LE=1
make -s
cp u-boot-db78200_MP.bin ../images/DB-78200-A-BP/LE/CV
cp u-boot-db78200_MP.srec ../images/DB-78200-A-BP/LE/CV
cp u-boot-db78200_MP ../images/DB-78200-A-BP/LE/CV

make mrproper
make db78200_MP_config LE=1 LNX=1
make -s
cp u-boot-db78200_MP.bin ../images/DB-78200-A-BP/LE/DUAL_LINUX
cp u-boot-db78200_MP.srec ../images/DB-78200-A-BP/LE/DUAL_LINUX
cp u-boot-db78200_MP ../images/DB-78200-A-BP/LE/DUAL_LINUX

make mrproper
make db78200_MP_config BE=1
make -s
cp u-boot-db78200_MP.bin ../images/DB-78200-A-BP/BE/CV
cp u-boot-db78200_MP.srec ../images/DB-78200-A-BP/BE/CV
cp u-boot-db78200_MP ../images/DB-78200-A-BP/BE/CV

make mrproper
make db78200_MP_config BE=1 LNX=1
make -s
cp u-boot-db78200_MP.bin ../images/DB-78200-A-BP/BE/DUAL_LINUX
cp u-boot-db78200_MP.srec ../images/DB-78200-A-BP/BE/DUAL_LINUX
cp u-boot-db78200_MP ../images/DB-78200-A-BP/BE/DUAL_LINUX

make mrproper
make rd78200_MP_AMC_config LE=1 SPIBOOT=1
make -s
cp u-boot-rd78200_MP_AMC.bin ../images/RD-78200-A-AMC/LE/
cp u-boot-rd78200_MP_AMC.srec ../images/RD-78200-A-AMC/LE/
cp u-boot-rd78200_MP_AMC ../images/RD-78200-A-AMC/LE/

