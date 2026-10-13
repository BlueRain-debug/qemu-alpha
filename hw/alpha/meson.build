alpha_ss = ss.source_set()
alpha_ss.add(when: 'CONFIG_ALPHASERVER_2100A', if_true: files('alpha2100a.c', 'alpha2100a-i2c.c'))
alpha_ss.add(when: 'CONFIG_JENSEN_S3', if_true: files('jensen-s3.c'))
alpha_ss.add(when: 'CONFIG_JENSEN', if_true: files(
  'jensen.c', 'jensen-scsi.c', 'jensen-mpu401.c'))
alpha_ss.add(when: 'CONFIG_CLIPPER', if_true: files('clipper.c', 'rmc.c', 'tigcontrol.c'))
alpha_ss.add(when: 'CONFIG_TSUNAMI', if_true: files('tsunami.c'))
alpha_ss.add(when: 'CONFIG_TIGBUS', if_true: files('tigbus.c'))
alpha_ss.add(when: 'CONFIG_AM29F016', if_true: files('am29f016.c'))

hw_arch += {'alpha': alpha_ss}
