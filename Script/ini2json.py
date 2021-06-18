import json
from configparser import ConfigParser

filename = './config.ini'
jsonfile = 'mysql.json'

cfg = configparser()
cfg.read(filename)

dest = {}

for section in cfg.sections():
	dest[section] = dict(cfg.items(section)) #将每个模块当作字典的元素

with open(jsonfile, 'w') as f:
	json.dump(dest, f)

print(json.dumps(cfg._sections))
