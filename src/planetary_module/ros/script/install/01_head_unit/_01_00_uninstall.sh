#!/bin/bash

echo "==================================="
echo "99_uninstall.sh"
echo "==================================="

sudo apt purge -y libreoffice*
sudo apt purge -y gnome-sudoku gnome-todo cheese leafpad gnome-mahjongg thunderbird aisleriot
sudo apt purge -y libgnome-games-support-common
sudo apt purge -y youtube-dl

sudo apt purge -y fluid

sudo apt purge -y unity-lens-photos shotwell remmina rhythmbox
sudo apt purge -y printer-driver-*
sudo apt purge -y system-config-printer*
sudo apt purge -y openprinting-ppds ippusbxd bsdmainutils
sudo apt purge -y foomatic-db-compressed-ppds policykit-desktop-privileges

sudo apt install -y gnome-control-center

sudo apt autoremove -y

