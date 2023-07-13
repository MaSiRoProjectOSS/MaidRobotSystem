#!/bin/bash

echo "==================================="
echo "02_install_mysql.sh"
echo "==================================="

## update apt
sudo apt update
sudo apt upgrade -y

## =======================================
cd `dirname $0`
## PHP & mysql install
sudo apt-get install -y php mysql-server mysql-client

## mysql sql setting
sudo mysql_secure_installation

cat file/mysqlsetting.txt | sudo sh -c 'cat -  >> /etc/mysql/my.cnf'
sudo /etc/init.d/mysql restart

## sql start
sudo service mysql start

## =======================================
## SQL のデータベース作成
sudo mysql -u root < file/mysql_CreateDB.txt

sudo apt-get install -y phpmyadmin
