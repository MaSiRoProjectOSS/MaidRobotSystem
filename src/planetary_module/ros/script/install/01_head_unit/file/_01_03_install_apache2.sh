#!/bin/bash

echo "==================================="
echo "01_03_install_apache2.sh"
echo "==================================="

# ///////////////////////////////////////////////////////////////////
## =======================================
## 設定
if [ -z "`echo ${MASIRO_PROJECT_ROOT_FOLDER}`" ]; then
    MY_PROJECT=/opt/masiro_ros_framework
else
    MY_PROJECT=${MASIRO_PROJECT_ROOT_FOLDER}
fi
MY_SCRIPT_WS=`echo ${MY_PROJECT}`/ros_workspace/src/Jetson/script/install
MY_HTML_WS=`echo ${MY_PROJECT}`/ros_workspace/src/Jetson/www/html
# ///////////////////////////////////////////////////////////////////
`echo ${MY_SCRIPT_WS}`
## update apt
sudo apt update
sudo apt upgrade -y

## =======================================
## WebServer化
sudo apt-get install -y apache2 openssl php
if [ -z "`cat /etc/apache2/conf-available/fqdn.conf | grep ${HOSTNAME}`" ]; then
    sudo touch /etc/apache2/conf-available/fqdn.conf
    echo ServerName ${HOSTNAME} | sudo sh -c 'cat - > /etc/apache2/conf-available/fqdn.conf'
fi

sudo a2enconf fqdn

## 証明書は「/etc/ssl/certs」にインストールされるが、専用のパスに生成します。
sudo mkdir -p /etc/ssl/localcerts

## 証明書の作成、必要な設定を質問されるので、自身の情報を記載してください。
### SSLはコピーにするように変更
sudo cp `echo ${MY_SCRIPT_WS}`/file/ssl-cert-snakeoil.pem /etc/ssl/localcerts/apache-selfsigned.pem
sudo cp `echo ${MY_SCRIPT_WS}`/file/ssl-cert-snakeoil.key /etc/ssl/localcerts/apache-selfsigned.key
# sudo openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout /etc/ssl/localcerts/apache-selfsigned.key -out /etc/ssl/localcerts/apache-selfsigned.pem

## 本来はrootしかみれない600だが、当バージョンではrosが閲覧できるように権限を持たせる。
sudo chmod 644 /etc/ssl/localcerts/apache*

# SSLの有効化
sudo a2enmod ssl
sudo a2ensite default-ssl

# サイトの有効化
## 設定ミスの時ののバックアップ
sudo cp /etc/apache2/sites-available/default-ssl.conf /etc/apache2/sites-available/default-ssl.bak

# 構成スニペットの作成
if [ ! -e "/etc/apache2/conf-available/ssl-params.conf" ]; then
    cat file/ssl-params.conf | sudo sh -c 'cat - > /etc/apache2/conf-available/ssl-params.conf'
fi

# パスを変更
if [ ! -d "/var/www/html/IoMs" ]; then
    sudo mv /var/www/html /var/www/html2
    sudo ln -s `echo ${MY_HTML_WS}` /var/www/html
fi

sudo rm /etc/ssl/certs/ssl-cert-snakeoil.pem
sudo rm /etc/ssl/private/ssl-cert-snakeoil.key

sudo cp /etc/ssl/localcerts/apache-selfsigned.pem /etc/ssl/certs/ssl-cert-snakeoil.pem
sudo cp /etc/ssl/localcerts/apache-selfsigned.key /etc/ssl/private/ssl-cert-snakeoil.key

APACHE2_DEFAULT_CONF=/etc/apache2/sites-available/000-default.conf
if [ -z "`cat ${APACHE2_DEFAULT_CONF} | grep Redirect`" ]; then
    APACHE2_DEFAULT_CONF_LINENUM=$((1 + `grep -e "DocumentRoot " -n ${APACHE2_DEFAULT_CONF} | sed -e 's/:.*//g'`))
    sudo sed -i -e "${APACHE2_DEFAULT_CONF_LINENUM}a Redirect / https://" ${APACHE2_DEFAULT_CONF}
fi

sudo service apache2 restart
