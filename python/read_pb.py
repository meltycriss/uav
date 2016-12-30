import scene_pb2 

s = ''
currS = ''

#with open('./data.txt', 'r') as f:
#    for s in f:
#        if(s[:-1]=='###'):
#            print 'true'
#        else:
#            print 'false'
#

#print '-----------------------'
#
#with open('./data.txt', 'r') as f:
#    while(True):
#        line = f.readline()
#        if not line: break
#        print line,
#
#print '-----------------------'
#
#with open('./data.txt', 'r') as f:
#    s = f.read()
#    print s
#
#print '-----------------------'

with open('./scenes.txt', 'r') as f:
    for line in f:
        if(line[:-1]=='###'):
            scene = scene_pb2.Scene()
            # the damn last \n
            scene.ParseFromString(s[:-1])
            print (scene)
            s = ''
        else:
            s += line

#print (s)


#request=message_pb2.Request()
#request.userid=1234
#request.type=1
#request.desc="i am a user"
#
#request_str=request.SerializeToString()
#
#print request_str
#
#print "receiver:"
#response=message_pb2.Request()
#response.ParseFromString(request_str)
#print response.userid

#person = x_pb2.Person()
#person.name = 'criss'
#person.id = 1446
#person.email = 'criss.lin@hotmail.com'
#
#person_str = person.SerializeToString()
#
#with open('py.txt', 'w') as f:
#    f.write(person_str)
#
#person_str = ''
#with open('./py.txt', 'r') as f:
#    person_str = f.read()
#
#person = x_pb2.Person()
#person.ParseFromString(person_str)
#print (person.name + ' ' + str(person.id) + ' ' + person.email)
