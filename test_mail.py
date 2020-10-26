#!/usr/bin/python3
# -*- coding: utf-8 -*-
import smtplib, ssl
from PyQt5.QtCore import QSettings

mail_settings = QSettings("mail.ini", QSettings.IniFormat)        
port = 465  # For SSL
context = ssl.create_default_context()  # Create a secure SSL context
with smtplib.SMTP_SSL("smtp.gmail.com", port, context=context) as server:
    login = mail_settings.value('smtp/login')
    password = mail_settings.value('smtp/password')
    server.login(login, password)

    message = """Subject: Progress = {}% \n\n Still {} s left""".format(30, 3341423)
    # do something fancy here in future: https://realpython.com/python-send-email/#sending-fancy-emails
    server.sendmail(mail_settings.value('smtp/login'), \
                    mail_settings.value('receiver/email'), \
                    message)
