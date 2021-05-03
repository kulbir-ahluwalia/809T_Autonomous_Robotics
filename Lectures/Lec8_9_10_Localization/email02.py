import email
import time
import imaplib

def checkEmail():
    mail = imaplib.IMAP4_SSL('imap.gmail.com');
    mail.login('enpm809t.kulbir@gmail.com','809Tenpm.123');

    mail.list();


    count = 0

    while count<60:

        try:
            mail.select("inbox");

            result, data = mail.search(None,'(UNSEEN FROM "ENPM809TS19@gmail.com")');

            print(result)
            print(len(data))

            ids = data[0]
            # print(ids)

            id_list = ids.split()

            latest_email_id = id_list[-1]
            result,data = mail.fetch(latest_email_id,"(RFC822)");

            if data is None:
                print("Waiting....")

            if data is not None:
                print("Process initiated!!")

        except IndexError:
            time.sleep(2)

            if count < 59:
                count = count +1
                continue

            else:
                print("Gameover")
                count =60


checkEmail()




























# pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
# command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + pic_time + '.jpg'
# os.system(command)

# #EMAIL
# smtpUser = 'enpm809t.kulbir@gmail.com'
# smtpPass = '809Tenpm.123'

# #DESTINATION
# toAdd = 'kulbir.ahluwalia97@gmail.com'
# # toAdd = 'ENPM809TS19@gmail.com'
# fromAdd = smtpUser
# # subject = 'ENPM809T_picture_JBL_speaker_Kulbir' + pic_time
# subject = 'ENPM809T_picture_JBL_speaker_Kulbir'

# msg = MIMEMultipart()
# msg['Subject'] = subject
# msg['From'] = fromAdd
# msg['To'] = toAdd
# # msg.preamble = "IMage recorded at : " + pic_time

# #EMAIL TEXT
# # body = MIMEText("Image recorded at : " + pic_time)
# body = MIMEText("Name: Kulbir Singh Ahluwalia, Image: Golder JBL speaker!! ")
# msg.attach(body)

# fp = open('JBL_Kulbir'+'.jpg','rb')
# img = MIMEImage(fp.read())
# fp.close()
# msg.attach(img)

# #send email

# s = smtplib.SMTP('smtp.gmail.com',587)

# s.ehlo()
# s.starttls()
# s.ehlo()

# s.login(smtpUser,smtpPass)
# s.sendmail(fromAdd, toAdd, msg.as_string())
# s.quit()

# print("Email DELIVERED!!!!!")