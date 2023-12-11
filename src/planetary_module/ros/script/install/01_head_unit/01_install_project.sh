#!/bin/bash

cd `dirname $0`

bash file/_01_00_uninstall.sh

bash file/_01_02_install_python.sh
bash file/_01_03_install_apache2.sh
bash file/_01_06_install_software.sh

sudo apt autoremove -y

echo "==================================="
echo "Fin."
echo "==================================="
