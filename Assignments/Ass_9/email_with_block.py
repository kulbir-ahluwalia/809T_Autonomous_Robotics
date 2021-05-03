import os
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
# command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + pic_time + '.jpg'
command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + 'green_block_kulbir' + '.jpg'
os.system(command)

#EMAIL
smtpUser = 'enpm809t.kulbir@gmail.com'
smtpPass = '809Tenpm.123'

#DESTINATION
# toAdd = 'kulbir97ahluwalia@gmail.com'
toAdd = ['kulbir97ahluwalia@gmail.com','kulbir.ahluwalia97@gmail.com']
# toAdd = 'ENPM809TS19@gmail.com'
fromAdd = smtpUser
# subject = 'ENPM809T_picture_JBL_speaker_Kulbir' + pic_time
subject = 'ENPM809T_picture_green_blocked_retrieved_Kulbir'

msg = MIMEMultipart()
msg['Subject'] = subject
msg['From'] = fromAdd
msg['To'] = toAdd
msg['To'] = ','.join(toAdd)
msg.preamble = "Image recorded of "


# msg.preamble = "IMage recorded at : " + pic_time

#EMAIL TEXT
# body = MIMEText("Image recorded at : " + pic_time)
body = MIMEText("Name: Kulbir Singh Ahluwalia, Image: green block retrieved!! ")
msg.attach(body)

fp = open('green_block_kulbir'+'.jpg','rb')
img = MIMEImage(fp.read())
fp.close()
msg.attach(img)

#send email

s = smtplib.SMTP('smtp.gmail.com',587)

s.ehlo()
s.starttls()
s.ehlo()

s.login(smtpUser,smtpPass)
s.sendmail(fromAdd, toAdd, msg.as_string())
s.quit()

print("Email DELIVERED!!!!!")











import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage



def sendEmail(picName):
    # Email information
    smtpUser = 'enpm809t.kulbir@gmail.com'
    smtpPass = '809Tenpm.123'

    # Destination email information
    toAdd = ['spatan07@terpmail.umd.edu', 'ENPM809TS19@gmail.com', 'skotasai@umd.edu']
    fromAdd = smtpUser
    subject = 'Assignment output: Image recorded of '+ picName
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    #msg['To'] = toAdd
    msg['To'] = ",".join(toAdd)
    msg.preamble = "Image recorded of "+picName
    
    #Email Text
    body = MIMEText("Image recorded of "+ picName)
    msg.attach(body)

    #Attach image
    fp = open(picName+'.jpg','rb')
    img = MIMEImage(fp.read())
    fp.close()
    msg.attach(img)
    
    #send email
    s = smtplib.SMTP('smtp.gmail.com',587)

    s.ehlo()
    s.starttls()
    s.ehlo()
    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, toAdd, msg.as_string())
    s.quit()
    
    print('Email Sent!')