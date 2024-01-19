#!/bin/bash

echo "==================================="
echo "99_uninstall.sh"
echo "==================================="

# office
sudo apt purge -y libreoffice*

# browser
sudo apt purge -y youtube-dl

# FLTK
sudo apt purge -y fluid

# Graphis
sudo apt purge -y unity-lens-photos shotwell remmina rhythmbox

# printer
sudo apt purge -y printer-driver-* system-config-printer* openprinting-ppds

#
sudo apt purge -y ippusbxd bsdmainutils
sudo apt purge -y foomatic-db-compressed-ppds policykit-desktop-privileges

# gnome
## game
sudo apt purge -y gnome-sudoku gnome-todo cheese leafpad gnome-mahjongg thunderbird aisleriot
sudo apt purge -y libgnome-games-support-common
## cotrol-center
sudo apt install -y gnome-control-center

sudo apt autoremove -y
sudo apt full-upgrade -y

