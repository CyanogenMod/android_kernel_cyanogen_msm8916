	.cpu generic+fp+simd
	.file	"gsl_point_id_arm64_20150205.c"
	.local	point_array
	.comm	point_array,1200,8
	.local	point_pointer
	.comm	point_pointer,80,8
	.local	point_stretch
	.comm	point_stretch,80,8
	.local	point_report
	.comm	point_report,80,8
	.local	point_now
	.comm	point_now,40,8
	.local	point_delay
	.comm	point_delay,40,8
	.local	filter_deep
	.comm	filter_deep,40,8
	.local	point_edge
	.comm	point_edge,12,8
	.local	point_decimal
	.comm	point_decimal,40,8
	.local	pressure_now
	.comm	pressure_now,40,8
	.local	pressure_array
	.comm	pressure_array,320,8
	.local	pressure_report
	.comm	pressure_report,40,8
	.local	pressure_pointer
	.comm	pressure_pointer,64,8
	.local	global_state
	.comm	global_state,4,8
	.local	inte_count
	.comm	inte_count,4,4
	.local	csensor_count
	.comm	csensor_count,4,4
	.local	point_n
	.comm	point_n,4,4
	.local	point_num
	.comm	point_num,4,4
	.local	prev_num
	.comm	prev_num,4,4
	.local	point_near
	.comm	point_near,4,4
	.local	point_shake
	.comm	point_shake,4,4
	.local	reset_mask_send
	.comm	reset_mask_send,4,4
	.local	reset_mask_max
	.comm	reset_mask_max,4,4
	.local	reset_mask_count
	.comm	reset_mask_count,4,4
	.local	global_flag
	.comm	global_flag,4,8
	.local	id_flag
	.comm	id_flag,4,8
	.local	id_first_coe
	.comm	id_first_coe,4,4
	.local	id_speed_coe
	.comm	id_speed_coe,4,4
	.local	id_static_coe
	.comm	id_static_coe,4,4
	.local	average
	.comm	average,4,4
	.local	soft_average
	.comm	soft_average,4,4
	.local	report_delay
	.comm	report_delay,4,4
	.local	delay_key
	.comm	delay_key,4,4
	.local	report_ahead
	.comm	report_ahead,4,4
	.local	report_delete
	.comm	report_delete,4,4
	.local	median_dis
	.comm	median_dis,4,8
	.local	shake_min
	.comm	shake_min,4,4
	.local	match_y
	.comm	match_y,8,8
	.local	match_x
	.comm	match_x,8,8
	.local	ignore_y
	.comm	ignore_y,8,8
	.local	ignore_x
	.comm	ignore_x,8,8
	.local	screen_y_max
	.comm	screen_y_max,4,4
	.local	screen_x_max
	.comm	screen_x_max,4,4
	.local	point_num_max
	.comm	point_num_max,4,4
	.local	drv_num
	.comm	drv_num,4,4
	.local	sen_num
	.comm	sen_num,4,4
	.local	drv_num_nokey
	.comm	drv_num_nokey,4,4
	.local	sen_num_nokey
	.comm	sen_num_nokey,4,4
	.local	coordinate_correct_able
	.comm	coordinate_correct_able,4,4
	.local	coordinate_correct_coe_x
	.comm	coordinate_correct_coe_x,256,8
	.local	coordinate_correct_coe_y
	.comm	coordinate_correct_coe_y,256,8
	.local	edge_cut
	.comm	edge_cut,16,8
	.local	stretch_array
	.comm	stretch_array,128,8
	.local	stretch_active
	.comm	stretch_active,128,8
	.local	shake_all_array
	.comm	shake_all_array,64,8
	.local	edge_start
	.comm	edge_start,4,4
	.local	reset_mask_dis
	.comm	reset_mask_dis,4,4
	.local	reset_mask_type
	.comm	reset_mask_type,4,4
	.local	key_map_able
	.comm	key_map_able,4,4
	.local	key_range_array
	.comm	key_range_array,96,8
	.local	filter_able
	.comm	filter_able,4,4
	.local	filter_coe
	.comm	filter_coe,16,8
	.local	multi_x_array
	.comm	multi_x_array,16,8
	.local	multi_y_array
	.comm	multi_y_array,16,8
	.local	multi_group
	.comm	multi_group,1024,8
	.local	ps_coe
	.comm	ps_coe,128,8
	.local	pr_coe
	.comm	pr_coe,128,8
	.local	point_repeat
	.comm	point_repeat,8,8
	.local	near_set
	.comm	near_set,8,8
	.local	diagonal
	.comm	diagonal,4,4
	.local	point_extend
	.comm	point_extend,4,4
	.local	edge_first
	.comm	edge_first,4,4
	.local	edge_first_coe
	.comm	edge_first_coe,4,4
	.local	point_corner
	.comm	point_corner,4,4
	.local	config_static
	.comm	config_static,2048,8
	.local	gesture_num
	.comm	gesture_num,4,4
	.local	gesture_num_last
	.comm	gesture_num_last,4,4
	.local	gesture_dis_min
	.comm	gesture_dis_min,4,4
	.local	gesture_deal
	.comm	gesture_deal,4,4
	.local	gesture_last
	.comm	gesture_last,4,4
	.local	gesture_value
	.comm	gesture_value,4,4
	.local	gesture_threshold
	.comm	gesture_threshold,8,8
	.local	x_scale
	.comm	x_scale,4,4
	.local	y_scale
	.comm	y_scale,4,4
	.local	click_count
	.comm	click_count,16,8
	.local	point_click
	.comm	point_click,16,8
	.local	double_click
	.comm	double_click,4,4
	.local	double_down
	.comm	double_down,4,4
	.local	double_up
	.comm	double_up,4,4
	.local	gesture_min_x
	.comm	gesture_min_x,4,4
	.local	gesture_min_y
	.comm	gesture_min_y,4,4
	.local	size_ratio
	.comm	size_ratio,4,4
	.local	multi_x
	.comm	multi_x,40,8
	.local	multi_o
	.comm	multi_o,40,8
	.local	gesture_multi
	.comm	gesture_multi,4,4
	.local	multi_set
	.comm	multi_set,4,4
	.local	gesture_flag
	.comm	gesture_flag,4,4
	.local	model_extern
	.comm	model_extern,8,8
	.local	model_extern_len
	.comm	model_extern_len,4,4
	.local	ReadIICInt
	.comm	ReadIICInt,8,8
	.section	.rodata
	.align	3
	.type	model_default, %object
	.size	model_default, 5688
model_default:
	.word	16
	.word	51
	.word	924254469
	.word	2019033088
	.word	-1164375549
	.word	-304533996
	.word	-1177005241
	.word	2006476123
	.word	911700331
	.word	521497972
	.word	1614834025
	.word	-1568575639
	.word	-473792395
	.word	-100943719
	.word	-1008475950
	.word	-2086340882
	.word	1113784316
	.word	2226173
	.word	16
	.word	54
	.word	-1564605440
	.word	1887969040
	.word	1113079854
	.word	489581652
	.word	68067460
	.word	167892668
	.word	941488620
	.word	1884618749
	.word	-1467155714
	.word	-624763409
	.word	-1002805
	.word	-487095145
	.word	-1429769094
	.word	1921938546
	.word	995524987
	.word	69244555
	.word	16
	.word	54
	.word	-335609854
	.word	-1042939648
	.word	-1733555194
	.word	1971724566
	.word	1399012142
	.word	893606217
	.word	522812522
	.word	202744719
	.word	314551
	.word	235139808
	.word	908197885
	.word	1531636985
	.word	2003487708
	.word	2038145462
	.word	1449888400
	.word	742494859
	.word	16
	.word	54
	.word	402694053
	.word	1227921050
	.word	2036437395
	.word	-1433235056
	.word	-607938413
	.word	-34555229
	.word	-352655919
	.word	-1076429836
	.word	-1901658113
	.word	1685580537
	.word	1398193365
	.word	1599639205
	.word	1969906038
	.word	-1803273652
	.word	-1180297175
	.word	-439549942
	.word	16
	.word	55
	.word	301989889
	.word	925106176
	.word	1531510784
	.word	-2140274688
	.word	-1517092864
	.word	-893976576
	.word	-270793984
	.word	-100851954
	.word	-487703248
	.word	-975940022
	.word	-1481151391
	.word	-1936030343
	.word	1904123795
	.word	1482931117
	.word	1129110473
	.word	1010696172
	.word	16
	.word	56
	.word	-620821748
	.word	-1900806144
	.word	1097272326
	.word	1847574
	.word	1126061128
	.word	-1888918691
	.word	-776631697
	.word	-824192849
	.word	-2001930778
	.word	1013121021
	.word	119066097
	.word	873763754
	.word	2052418941
	.word	-1046589600
	.word	-102425540
	.word	-1243873275
	.word	16
	.word	56
	.word	1652368158
	.word	793327660
	.word	51862088
	.word	554271090
	.word	1446811020
	.word	-2056343390
	.word	-1885742396
	.word	1534521061
	.word	608173557
	.word	151903220
	.word	689482946
	.word	1295680663
	.word	2019778930
	.word	-1550958512
	.word	-793108690
	.word	-1638385
	.word	16
	.word	56
	.word	1985246345
	.word	-1147646366
	.word	-371781581
	.word	-1648164859
	.word	1349910784
	.word	187244040
	.word	705255747
	.word	1934522472
	.word	-1130982008
	.word	-85934172
	.word	-755632669
	.word	-2052325637
	.word	945747710
	.word	35046630
	.word	789483173
	.word	2003922555
	.word	16
	.word	57
	.word	-385939691
	.word	-1261567999
	.word	-2120612608
	.word	1332221196
	.word	506866206
	.word	807996
	.word	655192425
	.word	1531015795
	.word	-1871288723
	.word	-1113109926
	.word	-573628874
	.word	-925421029
	.word	-1262591157
	.word	-1532193155
	.word	-1784886608
	.word	-1987051548
	.word	16
	.word	65
	.word	-1362493429
	.word	1955597826
	.word	1096297755
	.word	405428546
	.word	34182005
	.word	100716465
	.word	739638506
	.word	1716187646
	.word	-1786853155
	.word	-1247243088
	.word	-910206858
	.word	-674030534
	.word	-724086476
	.word	-741110417
	.word	-606811733
	.word	-1442586
	.word	16
	.word	65
	.word	-1901395710
	.word	1450251015
	.word	742341157
	.word	236677969
	.word	304777
	.word	235068866
	.word	1042480114
	.word	1918626042
	.word	-1836796208
	.word	-1499627621
	.word	-1280424606
	.word	-1212675288
	.word	-1482282740
	.word	-1314233530
	.word	-792816515
	.word	-1590361
	.word	16
	.word	65
	.word	1534656774
	.word	691872780
	.word	236543553
	.word	33987966
	.word	67165373
	.word	857078007
	.word	1565182178
	.word	2070712493
	.word	-1903864974
	.word	-1684728780
	.word	-1616955625
	.word	-1616939433
	.word	-1465666154
	.word	-1179651115
	.word	-489564689
	.word	-877896
	.word	16
	.word	66
	.word	1448679936
	.word	1196314165
	.word	876577128
	.word	606909852
	.word	253487054
	.word	386784244
	.word	1009104068
	.word	1851038886
	.word	-1551329383
	.word	-658661994
	.word	-51265880
	.word	-739646252
	.word	-1598359824
	.word	1803943162
	.word	911278077
	.word	1833983
	.word	16
	.word	66
	.word	202120960
	.word	303062847
	.word	202350463
	.word	34135999
	.word	33608177
	.word	453744050
	.word	1278241661
	.word	-1939054237
	.word	-911511454
	.word	-186604921
	.word	-251667007
	.word	-1244267793
	.word	1972763903
	.word	911600886
	.word	152748252
	.word	252149920
	.word	16
	.word	66
	.word	-1076556542
	.word	-1800850400
	.word	1769888584
	.word	1062437487
	.word	287811219
	.word	402825909
	.word	1261602195
	.word	-2107215239
	.word	-1147179411
	.word	-271088009
	.word	-151143519
	.word	-891034670
	.word	-1800341263
	.word	1534590973
	.word	624880634
	.word	286569938
	.word	16
	.word	67
	.word	-83942601
	.word	-622065389
	.word	-1329266685
	.word	-2019818240
	.word	1601310473
	.word	994846490
	.word	489375028
	.word	135359062
	.word	234367
	.word	168017064
	.word	622190030
	.word	1262023400
	.word	1969291257
	.word	-1635125251
	.word	-961353488
	.word	-287713313
	.word	16
	.word	67
	.word	-1396768512
	.word	-2036790780
	.word	1685396242
	.word	1163080229
	.word	725045569
	.word	388067680
	.word	135239042
	.word	17021864
	.word	33612495
	.word	537917937
	.word	1177812732
	.word	1817837567
	.word	-1820263171
	.word	-1180240647
	.word	-590617363
	.word	-1126694
	.word	16
	.word	67
	.word	1466370560
	.word	977871126
	.word	573390126
	.word	253189709
	.word	67666287
	.word	108179
	.word	50383544
	.word	235400668
	.word	790494971
	.word	1380054524
	.word	1935860460
	.word	-1904162346
	.word	-1466191940
	.word	-1011510111
	.word	-506298743
	.word	-1022349
	.word	16
	.word	68
	.word	-1716157041
	.word	1601996931
	.word	675454095
	.word	17811115
	.word	604828897
	.word	1564606203
	.word	-1787104772
	.word	-1045574688
	.word	-758345041
	.word	-741189515
	.word	-673767621
	.word	-673768702
	.word	-724085448
	.word	-724267150
	.word	-505820756
	.word	-1641242
	.word	16
	.word	69
	.word	958174351
	.word	1935054229
	.word	-1383036270
	.word	-574068866
	.word	-236048307
	.word	-1008923365
	.word	-1985544190
	.word	1399657220
	.word	591084069
	.word	135491667
	.word	50440845
	.word	571399366
	.word	1463482350
	.word	-1854603268
	.word	-894503169
	.word	-1713173
	.word	16
	.word	71
	.word	-1429602304
	.word	1770651648
	.word	709431558
	.word	999974
	.word	587624545
	.word	1682142836
	.word	-1551669132
	.word	-624933031
	.word	-85193942
	.word	-17088481
	.word	-32416
	.word	-167919455
	.word	-907415841
	.word	-1985347591
	.word	1214905087
	.word	153674739
	.word	16
	.word	71
	.word	-908066555
	.word	-1699672056
	.word	1870867484
	.word	1381847868
	.word	1985965160
	.word	-1500619932
	.word	-759413422
	.word	-119135179
	.word	-218479058
	.word	-538282915
	.word	-992829815
	.word	-1531591245
	.word	-2137789991
	.word	1382809335
	.word	557445884
	.word	1034976
	.word	16
	.word	71
	.word	-352381919
	.word	-1177419000
	.word	-2002780160
	.word	1466961669
	.word	725558808
	.word	152522039
	.word	229222
	.word	218476439
	.word	706403015
	.word	1497427429
	.word	-1955400973
	.word	-1130042898
	.word	-455947298
	.word	-68117325
	.word	-168243531
	.word	-370016282
	.word	16
	.word	72
	.word	50467584
	.word	101005862
	.word	185229389
	.word	185304948
	.word	168537754
	.word	84333761
	.word	195560
	.word	285532400
	.word	773768145
	.word	1346219445
	.word	2003084711
	.word	-1635081822
	.word	-994991200
	.word	-354898520
	.word	-470595
	.word	-1820
	.word	16
	.word	75
	.word	488251392
	.word	387604544
	.word	286564480
	.word	101638080
	.word	285270783
	.word	874555589
	.word	1867484053
	.word	-1349549435
	.word	-271612285
	.word	-503532640
	.word	-1597914437
	.word	1619052229
	.word	742443463
	.word	1816975569
	.word	-1400050977
	.word	-305332493
	.word	16
	.word	75
	.word	573839616
	.word	353915958
	.word	236032114
	.word	202165422
	.word	118093546
	.word	168278488
	.word	689472926
	.word	1530945904
	.word	-1871092136
	.word	1652329340
	.word	709339034
	.word	286111921
	.word	1294974908
	.word	-1989424444
	.word	-1012475439
	.word	-1970718
	.word	16
	.word	76
	.word	1061817344
	.word	892808220
	.word	741428541
	.word	640249437
	.word	455183998
	.word	320253855
	.word	152096704
	.word	324064
	.word	369491967
	.word	925368319
	.word	1481179135
	.word	2053766655
	.word	-1685390083
	.word	-1129579268
	.word	-573703429
	.word	-1115655
	.word	16
	.word	77
	.word	151052543
	.word	538419392
	.word	975790977
	.word	1313088579
	.word	1515728912
	.word	1801416527
	.word	1953607311
	.word	2037903054
	.word	-1937717787
	.word	-1583839579
	.word	-1163049114
	.word	-741930455
	.word	-572902122
	.word	-404654763
	.word	-185551724
	.word	-330540
	.word	16
	.word	77
	.word	234897946
	.word	336892778
	.word	337044411
	.word	403947752
	.word	538668951
	.word	791027271
	.word	1716333577
	.word	2054388820
	.word	2088488357
	.word	2121977077
	.word	-1954251868
	.word	-1668076717
	.word	-709686007
	.word	-320709323
	.word	-84562299
	.word	-131114
	.word	16
	.word	77
	.word	16836351
	.word	100905933
	.word	235503770
	.word	521555559
	.word	858332468
	.word	1564868617
	.word	2037329429
	.word	-2004721594
	.word	-1869918359
	.word	-1567154121
	.word	-894303478
	.word	-337761259
	.word	-101425338
	.word	-17001864
	.word	-14933
	.word	-1826
	.word	16
	.word	77
	.word	318975
	.word	67148219
	.word	369972854
	.word	841029173
	.word	1884160002
	.word	-1601228783
	.word	-1246858416
	.word	-1111836779
	.word	-1044320550
	.word	-1027495476
	.word	-926653049
	.word	-640802750
	.word	-152293877
	.word	-50630064
	.word	-33704043
	.word	-66343
	.word	16
	.word	77
	.word	227471
	.word	184764768
	.word	622206770
	.word	1329006607
	.word	1818373142
	.word	2004048964
	.word	2088339060
	.word	-1937549202
	.word	-1617614015
	.word	-1112865258
	.word	-422244096
	.word	-135184601
	.word	-50696361
	.word	-155769
	.word	-12361
	.word	-25
	.word	16
	.word	333
	.word	-234888961
	.word	-588733760
	.word	-925801854
	.word	-1144970429
	.word	-1431164405
	.word	-1616614838
	.word	-1835488887
	.word	-2138380087
	.word	1869922270
	.word	1634303903
	.word	1432043616
	.word	1179582752
	.word	909983519
	.word	674201182
	.word	354401438
	.word	720092
	.word	16
	.word	78
	.word	67168255
	.word	319535311
	.word	673089953
	.word	942692977
	.word	1363487811
	.word	1734155539
	.word	1751666230
	.word	1802076006
	.word	1919922327
	.word	2020991176
	.word	-1870731016
	.word	-1482769454
	.word	-1078751324
	.word	-775331977
	.word	-421908152
	.word	-917479
	.word	12
	.word	79
	.word	775893279
	.word	270425158
	.word	50892662
	.word	115368
	.word	503900122
	.word	1328938484
	.word	-2123825921
	.word	-1281626379
	.word	-624439585
	.word	-202792776
	.word	-17207929
	.word	-100713388
	.word	-622062810
	.word	-1430060792
	.word	2005926144
	.word	1180570631
	.word	12
	.word	79
	.word	-2039479797
	.word	-1197468921
	.word	-473025501
	.word	-118462900
	.word	-222335
	.word	-235221323
	.word	-823855899
	.word	-1699414021
	.word	1702887423
	.word	911010552
	.word	371441879
	.word	626343
	.word	100751731
	.word	605171265
	.word	1295518748
	.word	-2107178748
	.word	12
	.word	79
	.word	-790429165
	.word	-1665794048
	.word	1753353221
	.word	944710678
	.word	388320310
	.word	67928166
	.word	111770
	.word	319153358
	.word	1143732976
	.word	2019491838
	.word	-1416433159
	.word	-742273053
	.word	-236740167
	.word	-364920
	.word	-117490860
	.word	-706211804
	.word	12
	.word	79
	.word	1013386
	.word	151012701
	.word	722935344
	.word	1514276369
	.word	-1921777408
	.word	-1130031349
	.word	-472762332
	.word	-51223476
	.word	-83912834
	.word	-470760273
	.word	-1110251817
	.word	-1901593870
	.word	1534394110
	.word	776268278
	.word	169591510
	.word	33722281
	.word	12
	.word	79
	.word	68061809
	.word	184794276
	.word	739895252
	.word	1564737269
	.word	-1871185410
	.word	-1096229138
	.word	-540035381
	.word	-135428705
	.word	-16952981
	.word	-420077001
	.word	-1211103214
	.word	-2086862846
	.word	1332283137
	.word	540483345
	.word	68046388
	.word	164456
	.word	12
	.word	79
	.word	-2103703296
	.word	1315447573
	.word	607668788
	.word	135625825
	.word	177560
	.word	218361297
	.word	1126629373
	.word	2053107448
	.word	-1416376352
	.word	-708728899
	.word	-186222960
	.word	-67158694
	.word	-806610138
	.word	-1766647784
	.word	1585063197
	.word	675494703
	.word	12
	.word	79
	.word	941514344
	.word	1850954070
	.word	-1551347633
	.word	-658612650
	.word	-101873811
	.word	-100877663
	.word	-655563310
	.word	-1514079758
	.word	1888223229
	.word	995490814
	.word	337893357
	.word	435391
	.word	151154825
	.word	638927191
	.word	1345920558
	.word	2120679440
	.word	16
	.word	79
	.word	1568079613
	.word	759555572
	.word	186301908
	.word	17140387
	.word	33576560
	.word	453716285
	.word	1244794649
	.word	2103640069
	.word	-1365897467
	.word	-674942690
	.word	-186098106
	.word	-17001864
	.word	-151075668
	.word	-823858985
	.word	-1665729546
	.word	1753415422
	.word	16
	.word	79
	.word	2036334588
	.word	-1349190145
	.word	-574103568
	.word	-118575159
	.word	-165228
	.word	-117553825
	.word	-739763153
	.word	-1581578736
	.word	1803945472
	.word	995237391
	.word	371607345
	.word	34241120
	.word	45205
	.word	386326731
	.word	1227815664
	.word	2139422462
	.word	16
	.word	81
	.word	-907804145
	.word	-1918173184
	.word	1366231042
	.word	406069009
	.word	33969463
	.word	890793832
	.word	1901292399
	.word	-1433511581
	.word	-557499829
	.word	-17561308
	.word	-504414684
	.word	-1060081321
	.word	-1414222706
	.word	-1650136886
	.word	-978653187
	.word	-35596574
	.word	16
	.word	83
	.word	-1211170815
	.word	-2036464895
	.word	1433209093
	.word	658382610
	.word	51463212
	.word	537486938
	.word	1362653803
	.word	-2106951562
	.word	-1281654661
	.word	-540305013
	.word	-672089
	.word	-588063024
	.word	-1379470872
	.word	2090138615
	.word	1264844797
	.word	422772735
	.word	16
	.word	83
	.word	-874642944
	.word	-1464200695
	.word	-2053692390
	.word	1786068783
	.word	-2140383407
	.word	-1500291493
	.word	-860264862
	.word	-220171157
	.word	-50359166
	.word	-521096538
	.word	-1110457669
	.word	-1750347319
	.word	1938152919
	.word	1281419491
	.word	658175214
	.word	1376249
	.word	16
	.word	85
	.word	84746761
	.word	87868
	.word	50367601
	.word	285654950
	.word	790359002
	.word	1615520240
	.word	-2005680947
	.word	-1516662616
	.word	-1129291912
	.word	-842585018
	.word	-791672559
	.word	-909427164
	.word	-959941799
	.word	-909727603
	.word	-657269310
	.word	-1769482
	.word	16
	.word	86
	.word	150998784
	.word	420556831
	.word	656429632
	.word	875394403
	.word	1094358917
	.word	1330100648
	.word	1616370121
	.word	1969879530
	.word	-1803094785
	.word	-1449142811
	.word	-1196378939
	.word	-910126173
	.word	-607030140
	.word	-370978205
	.word	-168874175
	.word	-455907
	.word	16
	.word	86
	.word	117473908
	.word	386899344
	.word	656389802
	.word	942657730
	.word	1178594269
	.word	1397618682
	.word	1666964457
	.word	1902821838
	.word	-2105890892
	.word	-1836413286
	.word	-1550158206
	.word	-1230217622
	.word	-926988461
	.word	-623824837
	.word	-303818460
	.word	-786419
	.word	16
	.word	86
	.word	117641984
	.word	151533862
	.word	151609164
	.word	134776178
	.word	134785944
	.word	101241534
	.word	17103077
	.word	369682935
	.word	858116060
	.word	1329706434
	.word	1834916776
	.word	-1988394610
	.word	-1499961227
	.word	-994750629
	.word	-506317245
	.word	-1107159
	.word	24
	.word	86
	.word	134221568
	.word	252456735
	.word	303190080
	.word	319975520
	.word	370446721
	.word	437825953
	.word	471585474
	.word	538833634
	.word	858060798
	.word	1329719279
	.word	1834932447
	.word	-1971599152
	.word	-1499940928
	.word	-1011571025
	.word	-523070051
	.word	-1145715
	.word	16
	.word	342
	.word	-184611584
	.word	-420597985
	.word	-656453056
	.word	-925797790
	.word	-1161653117
	.word	-1397508444
	.word	-1633363769
	.word	-1885734422
	.word	2021910518
	.word	1718601686
	.word	1499440309
	.word	1230079125
	.word	943678580
	.word	657474132
	.word	371073330
	.word	17630225
	.word	16
	.word	87
	.word	100671232
	.word	286023487
	.word	471375743
	.word	673373886
	.word	1093788669
	.word	1431020739
	.word	1700553860
	.word	2020549700
	.word	-2072039665
	.word	-1970901169
	.word	-1836142961
	.word	-1634079026
	.word	-1163079442
	.word	-708275277
	.word	-337620109
	.word	-650445
	.word	16
	.word	87
	.word	83923324
	.word	369998758
	.word	656401360
	.word	1077080825
	.word	1414253535
	.word	1633329333
	.word	1835564426
	.word	2088019603
	.word	-1954164292
	.word	-1651378201
	.word	-1280716316
	.word	-994268996
	.word	-758415469
	.word	-489007510
	.word	-219468735
	.word	-524266
	.word	16
	.word	87
	.word	11531
	.word	94543
	.word	151172499
	.word	672592852
	.word	1480905969
	.word	2036963254
	.word	-1971039371
	.word	-2054549711
	.word	2139114024
	.word	2122223212
	.word	-1971137872
	.word	-1248396044
	.word	-456078617
	.word	-51082837
	.word	-113305
	.word	-100859869
	.word	24
	.word	87
	.word	402960
	.word	50420801
	.word	100962935
	.word	252299437
	.word	689502176
	.word	1363338491
	.word	1751036619
	.word	2003860631
	.word	-2055368329
	.word	-1752316501
	.word	-1264454690
	.word	-607398420
	.word	-337337667
	.word	-185570169
	.word	-84461999
	.word	-196581
	.word	16
	.word	87
	.word	201335819
	.word	538269245
	.word	858491248
	.word	1262578564
	.word	1532180561
	.word	1784874527
	.word	2020748564
	.word	-2122555578
	.word	-1954114951
	.word	-1752054613
	.word	-1549862946
	.word	-1230055954
	.word	-960585029
	.word	-657493879
	.word	-303874730
	.word	-652764
	.word	16
	.word	87
	.word	218116633
	.word	504718667
	.word	740595838
	.word	926075825
	.word	1161690852
	.word	1447939559
	.word	1734187956
	.word	2070964354
	.word	-1870383793
	.word	-1550122468
	.word	-1381486313
	.word	-1162714038
	.word	-926902403
	.word	-556501103
	.word	-236501922
	.word	-585429
	.word	16
	.word	87
	.word	201336328
	.word	605578052
	.word	925803652
	.word	1262544068
	.word	1750588409
	.word	-2123063620
	.word	-1550687617
	.word	-1028380602
	.word	-741533433
	.word	-707438279
	.word	-740976006
	.word	-774645062
	.word	-589967891
	.word	-354120275
	.word	-152023956
	.word	-193748
	.word	16
	.word	343
	.word	-134275072
	.word	-454205376
	.word	-707026559
	.word	-1110712128
	.word	-1599417357
	.word	-1869049165
	.word	-2071309710
	.word	-2139025103
	.word	2004696349
	.word	1668120158
	.word	1398521758
	.word	1011416542
	.word	657441250
	.word	303727266
	.word	51003745
	.word	131105
	.word	16
	.word	89
	.word	369101587
	.word	1143800579
	.word	1263023131
	.word	1632656971
	.word	-1854317470
	.word	-1129888431
	.word	-506452425
	.word	-51114988
	.word	-386844382
	.word	-605984431
	.word	-690382206
	.word	-824980558
	.word	-1329270559
	.word	2140732407
	.word	1315438333
	.word	490143231
	.word	16
	.word	89
	.word	184627019
	.word	1109659418
	.word	1229737005
	.word	1783460971
	.word	-1601809563
	.word	-759485888
	.word	-270266609
	.word	-67809745
	.word	-50360978
	.word	-420230483
	.word	-1194199582
	.word	2040069631
	.word	1130094322
	.word	1346021824
	.word	-1955819367
	.word	-894795640
	.word	16
	.word	90
	.word	806748672
	.word	1682572291
	.word	-1719729405
	.word	-843906558
	.word	-404222720
	.word	-992594398
	.word	-1682940854
	.word	1938128748
	.word	1348577427
	.word	691981499
	.word	1307357
	.word	756284669
	.word	1648948984
	.word	-1770196493
	.word	-877531665
	.word	-1707540
	.word	16
	.word	60
	.word	-251720960
	.word	-773776364
	.word	-1329583068
	.word	-1868481485
	.word	1904233540
	.word	1365269588
	.word	792751460
	.word	220101749
	.word	369462662
	.word	942119828
	.word	1464314274
	.word	2003351985
	.word	-1769551424
	.word	-1247356207
	.word	-708383517
	.word	-169541642
	.word	16
	.word	62
	.word	386336256
	.word	958928399
	.word	1497966367
	.word	2037004079
	.word	-1702213569
	.word	-1180018351
	.word	-657953950
	.word	-85361036
	.word	-470512251
	.word	-1043161963
	.word	-1615811420
	.word	2123283123
	.word	1567476674
	.word	1011669971
	.word	523103460
	.word	1114101
	.word	16
	.word	4097
	.word	257279
	.word	84068057
	.word	185180082
	.word	336623756
	.word	572215909
	.word	908799808
	.word	1396773405
	.word	2036728330
	.word	-1601371904
	.word	-1011673333
	.word	-640796120
	.word	-303931571
	.word	-185432462
	.word	-84497255
	.word	-142400
	.word	-1306
	.word	16
	.word	4098
	.word	419464317
	.word	1278379909
	.word	2137361288
	.word	-1315403381
	.word	-473202808
	.word	-118337688
	.word	-319283915
	.word	-1076297461
	.word	-1868163836
	.word	-2121906643
	.word	2088597856
	.word	2138811539
	.word	-2004623418
	.word	-1348993036
	.word	-590877958
	.word	-1130025
	.word	16
	.word	4099
	.word	1967330330
	.word	-676844009
	.word	-117993636
	.word	-957945409
	.word	1687681274
	.word	322483163
	.word	50483074
	.word	1243350560
	.word	-1350823935
	.word	-119908309
	.word	-352404341
	.word	-1697907738
	.word	946401023
	.word	169382596
	.word	655503455
	.word	-2058220015
	.word	16
	.word	4100
	.word	67121668
	.word	302747232
	.word	387247037
	.word	791132390
	.word	1127766921
	.word	1531708205
	.word	1785094194
	.word	2003877006
	.word	-1937590042
	.word	-1684775798
	.word	-1365114579
	.word	-1145610961
	.word	-910050676
	.word	-472531737
	.word	-236298355
	.word	-720335
	.word	16
	.word	4101
	.word	419430922
	.word	1295188992
	.word	2087000074
	.word	-1584447701
	.word	-1280544168
	.word	-1128749685
	.word	-1329997889
	.word	-1969160980
	.word	1500639997
	.word	1095484123
	.word	1128238760
	.word	1397382518
	.word	1902195270
	.word	-1652223970
	.word	-793377271
	.word	-1436663
	.word	16
	.word	4102
	.word	-553711852
	.word	-1698955262
	.word	1467550210
	.word	490156056
	.word	17263947
	.word	789551242
	.word	1917951132
	.word	-1298887780
	.word	-1547937941
	.word	1619088979
	.word	641825123
	.word	34517393
	.word	537257939
	.word	1682111476
	.word	-1484391425
	.word	-339089421
	.word	16
	.word	4103
	.word	-151058944
	.word	-454224884
	.word	-706927075
	.word	-976406994
	.word	-1279506882
	.word	-1565894834
	.word	-1818597280
	.word	-2088142480
	.word	1954318978
	.word	1684839571
	.word	1381739684
	.word	1128971700
	.word	859557830
	.word	573234645
	.word	303820518
	.word	655350
	.word	16
	.word	4104
	.word	150997248
	.word	420551443
	.word	673197348
	.word	959462709
	.word	1229082181
	.word	1515281750
	.word	1801612645
	.word	2104785780
	.word	-1903785086
	.word	-1651073901
	.word	-1398493531
	.word	-1129005130
	.word	-893070905
	.word	-606936615
	.word	-320605719
	.word	-655368
	.word	16
	.word	4105
	.word	167836415
	.word	487910900
	.word	774233317
	.word	1043778516
	.word	1313127106
	.word	1599580850
	.word	1869060770
	.word	2138540176
	.word	-1937410690
	.word	-1667996821
	.word	-1415360166
	.word	-1145945782
	.word	-876400583
	.word	-573366488
	.word	-286977767
	.word	-589814
	.word	16
	.word	4106
	.word	-184550657
	.word	-504696842
	.word	-774251034
	.word	-1043673643
	.word	-1262765373
	.word	-1565742928
	.word	-1835361887
	.word	-2088204399
	.word	1987933309
	.word	1718510443
	.word	1398559838
	.word	1145913931
	.word	859648570
	.word	606806314
	.word	337383192
	.word	655368
	.word	16
	.word	4107
	.word	805306888
	.word	-1872756224
	.word	-239002100
	.word	-1243198145
	.word	1434801747
	.word	841241942
	.word	-1822271658
	.word	-238848930
	.word	-1159026281
	.word	1502261936
	.word	254388913
	.word	1883285671
	.word	-777996114
	.word	-788992816
	.word	1872755957
	.word	239074559
	.word	16
	.word	4108
	.word	637539340
	.word	1716077094
	.word	-2105766554
	.word	1837160628
	.word	676330228
	.word	168475104
	.word	1092583575
	.word	-1905749630
	.word	-692813932
	.word	-101566112
	.word	-705167342
	.word	-1850792182
	.word	2122543948
	.word	2122170778
	.word	-1265567003
	.word	-2364930
	.word	16
	.word	4109
	.word	1989020672
	.word	1180575257
	.word	422528565
	.word	17265754
	.word	840403589
	.word	1733133706
	.word	-1686014851
	.word	-860595095
	.word	-136103091
	.word	-335737567
	.word	-1227882492
	.word	-1818351859
	.word	-2004001470
	.word	-2088397960
	.word	2088815022
	.word	1819672548
	.word	16
	.word	4111
	.word	234943743
	.word	790553836
	.word	1329517788
	.word	1868547276
	.word	-1887456324
	.word	-1365335125
	.word	-826370917
	.word	-253786739
	.word	-251694982
	.word	-807443864
	.word	-1380036006
	.word	-1919007926
	.word	1836922172
	.word	1281107502
	.word	742135070
	.word	203161611
	.text
	.align	2
	.type	SortBubble, %function
SortBubble:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	w1, [sp, 4]
	str	wzr, [sp, 20]
	str	wzr, [sp, 28]
	b	.L2
.L6:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 24]
	b	.L3
.L5:
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	str	w0, [sp, 20]
	ldrsw	x0, [sp, 24]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w1, [x0]
	ldr	w0, [sp, 20]
	cmp	w1, w0
	bge	.L4
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldrsw	x1, [sp, 24]
	lsl	x1, x1, 2
	ldr	x2, [sp, 8]
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldrsw	x0, [sp, 24]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w1, [sp, 20]
	str	w1, [x0]
.L4:
	ldr	w0, [sp, 24]
	add	w0, w0, 1
	str	w0, [sp, 24]
.L3:
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 4]
	cmp	w1, w0
	blt	.L5
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L2:
	ldr	w1, [sp, 28]
	ldr	w0, [sp, 4]
	cmp	w1, w0
	blt	.L6
	add	sp, sp, 32
	ret
	.size	SortBubble, .-SortBubble
	.align	2
	.type	Sqrt, %function
Sqrt:
	sub	sp, sp, #32
	str	w0, [sp, 12]
	str	wzr, [sp, 28]
	mov	w0, 14
	str	w0, [sp, 24]
	b	.L8
.L10:
	ldr	w0, [sp, 24]
	mov	w1, 1
	lsl	w1, w1, w0
	ldr	w0, [sp, 28]
	add	w1, w1, w0
	ldr	w0, [sp, 24]
	mov	w2, 1
	lsl	w2, w2, w0
	ldr	w0, [sp, 28]
	add	w0, w2, w0
	mul	w1, w1, w0
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bgt	.L9
	ldr	w0, [sp, 24]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [sp, 28]
	orr	w0, w1, w0
	str	w0, [sp, 28]
.L9:
	ldr	w0, [sp, 24]
	sub	w0, w0, #1
	str	w0, [sp, 24]
.L8:
	ldr	w0, [sp, 24]
	cmp	w0, wzr
	bge	.L10
	ldr	w0, [sp, 28]
	add	sp, sp, 32
	ret
	.size	Sqrt, .-Sqrt
	.align	2
	.type	PointRange, %function
PointRange:
	sub	sp, sp, #16
	str	w0, [sp, 12]
	str	w1, [sp, 8]
	str	w2, [sp, 4]
	str	w3, [sp]
	ldr	w0, [sp, 12]
	cmp	w0, wzr
	bgt	.L13
	ldr	w1, [sp, 12]
	ldr	w0, [sp, 4]
	cmp	w1, w0
	beq	.L14
	ldr	w1, [sp, 8]
	ldr	w0, [sp]
	sub	w1, w1, w0
	mov	w2, 1
	ldr	w0, [sp, 4]
	sub	w0, w2, w0
	mul	w1, w1, w0
	ldr	w2, [sp, 12]
	ldr	w0, [sp, 4]
	sub	w0, w2, w0
	sdiv	w1, w1, w0
	ldr	w0, [sp]
	add	w0, w1, w0
	str	w0, [sp, 8]
.L14:
	mov	w0, 1
	str	w0, [sp, 12]
.L13:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bgt	.L15
	ldr	w1, [sp, 12]
	ldr	w0, [sp, 4]
	cmp	w1, w0
	beq	.L16
	ldr	w1, [sp, 8]
	ldr	w0, [sp]
	sub	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w2, w0, 6
	ldr	w0, [sp, 4]
	sub	w0, w2, w0
	mul	w1, w1, w0
	ldr	w2, [sp, 12]
	ldr	w0, [sp, 4]
	sub	w0, w2, w0
	sdiv	w1, w1, w0
	ldr	w0, [sp]
	add	w0, w1, w0
	str	w0, [sp, 8]
.L16:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 12]
.L15:
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bgt	.L17
	ldr	w1, [sp, 8]
	ldr	w0, [sp]
	cmp	w1, w0
	beq	.L18
	ldr	w1, [sp, 12]
	ldr	w0, [sp, 4]
	sub	w1, w1, w0
	mov	w2, 1
	ldr	w0, [sp]
	sub	w0, w2, w0
	mul	w1, w1, w0
	ldr	w2, [sp, 8]
	ldr	w0, [sp]
	sub	w0, w2, w0
	sdiv	w1, w1, w0
	ldr	w0, [sp, 4]
	add	w0, w1, w0
	str	w0, [sp, 12]
.L18:
	mov	w0, 1
	str	w0, [sp, 8]
.L17:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 8]
	cmp	w1, w0
	bgt	.L19
	ldr	w1, [sp, 8]
	ldr	w0, [sp]
	cmp	w1, w0
	beq	.L20
	ldr	w1, [sp, 12]
	ldr	w0, [sp, 4]
	sub	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w2, w0, 6
	ldr	w0, [sp]
	sub	w0, w2, w0
	mul	w1, w1, w0
	ldr	w2, [sp, 8]
	ldr	w0, [sp]
	sub	w0, w2, w0
	sdiv	w1, w1, w0
	ldr	w0, [sp, 4]
	add	w0, w1, w0
	str	w0, [sp, 12]
.L20:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 8]
.L19:
	ldr	w0, [sp, 12]
	cmp	w0, wzr
	bgt	.L21
	mov	w0, 1
	str	w0, [sp, 12]
.L21:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bgt	.L22
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 12]
.L22:
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bgt	.L23
	mov	w0, 1
	str	w0, [sp, 8]
.L23:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 8]
	cmp	w1, w0
	bgt	.L24
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 8]
.L24:
	ldr	w0, [sp, 12]
	lsl	w1, w0, 16
	ldr	w0, [sp, 8]
	add	w0, w1, w0
	add	sp, sp, 16
	ret
	.size	PointRange, .-PointRange
	.align	2
	.type	PointCoor, %function
PointCoor:
	sub	sp, sp, #16
	str	wzr, [sp, 12]
	b	.L27
.L30:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 2]
	and	w0, w0, -128
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L28
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	ldr	w1, [x0, x1, lsl 2]
	mov	w0, -40961
	and	w2, w1, w0
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	w2, [x0, x1, lsl 2]
	b	.L29
.L28:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	ldr	w1, [x0, x1, lsl 2]
	mov	w0, 24575
	movk	w0, 0xfff, lsl 16
	and	w2, w1, w0
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	w2, [x0, x1, lsl 2]
.L29:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L27:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [sp, 12]
	cmp	w1, w0
	blt	.L30
	add	sp, sp, 16
	ret
	.size	PointCoor, .-PointCoor
	.align	2
	.type	PointRepeat, %function
PointRepeat:
	sub	sp, sp, #48
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L32
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	ldr	w0, [x0]
	sub	w1, w0, #1
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	str	w1, [x0]
.L32:
	adrp	x0, prev_num
	add	x0, x0, :lo12:prev_num
	ldr	w1, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w1, w0
	ble	.L33
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	mov	w1, 8
	str	w1, [x0]
.L33:
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L34
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	bne	.L35
.L34:
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L36
	mov	w0, 96
	str	w0, [sp, 36]
	b	.L38
.L36:
	mov	w0, 32
	str	w0, [sp, 36]
	b	.L38
.L35:
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L39
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	ldr	w0, [x0, 4]
	str	w0, [sp, 36]
	b	.L38
.L39:
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	ldr	w0, [x0]
	str	w0, [sp, 36]
.L38:
	str	wzr, [sp, 44]
	b	.L40
.L49:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 44]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	bne	.L41
	b	.L42
.L41:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L43
	b	.L42
.L43:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [sp, 36]
	sub	w0, w1, w0
	str	w0, [sp, 32]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [sp, 36]
	add	w0, w1, w0
	str	w0, [sp, 28]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [sp, 36]
	sub	w0, w1, w0
	str	w0, [sp, 24]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [sp, 36]
	add	w0, w1, w0
	str	w0, [sp, 20]
	ldr	w0, [sp, 44]
	add	w0, w0, 1
	str	w0, [sp, 40]
	b	.L44
.L48:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 40]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	bne	.L45
	b	.L46
.L45:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 40]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L47
	b	.L46
.L47:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 40]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [sp, 16]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 40]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [sp, 12]
	ldr	w1, [sp, 16]
	ldr	w0, [sp, 32]
	cmp	w1, w0
	ble	.L46
	ldr	w1, [sp, 16]
	ldr	w0, [sp, 28]
	cmp	w1, w0
	bge	.L46
	ldr	w1, [sp, 12]
	ldr	w0, [sp, 24]
	cmp	w1, w0
	ble	.L46
	ldr	w1, [sp, 12]
	ldr	w0, [sp, 20]
	cmp	w1, w0
	bge	.L46
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 40]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	add	w0, w2, w0
	add	w0, w0, 1
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 40]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	add	w0, w2, w0
	add	w0, w0, 1
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 40]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [sp, 44]
	sub	w0, w0, #1
	str	w0, [sp, 44]
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	mov	w1, 8
	str	w1, [x0]
	b	.L42
.L46:
	ldr	w0, [sp, 40]
	add	w0, w0, 1
	str	w0, [sp, 40]
.L44:
	ldr	w0, [sp, 40]
	cmp	w0, 9
	ble	.L48
.L42:
	ldr	w0, [sp, 44]
	add	w0, w0, 1
	str	w0, [sp, 44]
.L40:
	ldr	w0, [sp, 44]
	cmp	w0, 9
	ble	.L49
	add	sp, sp, 48
	ret
	.size	PointRepeat, .-PointRepeat
	.align	2
	.type	PointPointer, %function
PointPointer:
	sub	sp, sp, #16
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	ldr	w0, [x0]
	add	w1, w0, 1
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	str	w1, [x0]
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	ldr	w1, [x0]
	mov	w0, 7999
	cmp	w1, w0
	ble	.L51
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	str	wzr, [x0]
.L51:
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	ldr	w1, [x0]
	mov	w0, 10
	sdiv	w2, w1, w0
	mov	w0, w2
	lsl	w0, w0, 2
	add	w0, w0, w2
	lsl	w0, w0, 1
	sub	w0, w1, w0
	str	w0, [sp, 8]
	str	wzr, [sp, 12]
	b	.L52
.L55:
	ldrsw	x1, [sp, 8]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 3
	adrp	x1, point_array
	add	x1, x1, :lo12:point_array
	add	x2, x0, x1
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldrsw	x1, [sp, 12]
	str	x2, [x0, x1, lsl 3]
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bne	.L53
	mov	w0, 9
	str	w0, [sp, 8]
	b	.L54
.L53:
	ldr	w0, [sp, 8]
	sub	w0, w0, #1
	str	w0, [sp, 8]
.L54:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L52:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L55
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	ldr	w1, [x0]
	mov	w0, 10
	sdiv	w2, w1, w0
	mov	w0, w2
	lsl	w0, w0, 2
	add	w0, w0, w2
	lsl	w0, w0, 1
	sub	w0, w1, w0
	str	w0, [sp, 8]
	str	wzr, [sp, 12]
	b	.L56
.L59:
	ldr	w0, [sp, 8]
	add	w0, w0, 10
	sxtw	x1, w0
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 3
	adrp	x1, point_array
	add	x1, x1, :lo12:point_array
	add	x2, x0, x1
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [sp, 12]
	str	x2, [x0, x1, lsl 3]
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bne	.L57
	mov	w0, 9
	str	w0, [sp, 8]
	b	.L58
.L57:
	ldr	w0, [sp, 8]
	sub	w0, w0, #1
	str	w0, [sp, 8]
.L58:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L56:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L59
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	ldr	w1, [x0]
	mov	w0, 10
	sdiv	w2, w1, w0
	mov	w0, w2
	lsl	w0, w0, 2
	add	w0, w0, w2
	lsl	w0, w0, 1
	sub	w0, w1, w0
	str	w0, [sp, 8]
	str	wzr, [sp, 12]
	b	.L60
.L63:
	ldr	w0, [sp, 8]
	add	w0, w0, 20
	sxtw	x1, w0
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 3
	adrp	x1, point_array
	add	x1, x1, :lo12:point_array
	add	x2, x0, x1
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [sp, 12]
	str	x2, [x0, x1, lsl 3]
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bne	.L61
	mov	w0, 9
	str	w0, [sp, 8]
	b	.L62
.L61:
	ldr	w0, [sp, 8]
	sub	w0, w0, #1
	str	w0, [sp, 8]
.L62:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L60:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L63
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	ldr	w1, [x0]
	asr	w0, w1, 31
	lsr	w0, w0, 29
	add	w1, w1, w0
	and	w1, w1, 7
	sub	w0, w1, w0
	str	w0, [sp, 8]
	str	wzr, [sp, 12]
	b	.L64
.L67:
	ldrsw	x1, [sp, 8]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 3
	adrp	x1, pressure_array
	add	x1, x1, :lo12:pressure_array
	add	x2, x0, x1
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldrsw	x1, [sp, 12]
	str	x2, [x0, x1, lsl 3]
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bne	.L65
	mov	w0, 7
	str	w0, [sp, 8]
	b	.L66
.L65:
	ldr	w0, [sp, 8]
	sub	w0, w0, #1
	str	w0, [sp, 8]
.L66:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L64:
	ldr	w0, [sp, 12]
	cmp	w0, 7
	ble	.L67
	str	wzr, [sp, 8]
	str	wzr, [sp, 12]
	b	.L68
.L70:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L69
	ldr	w0, [sp, 8]
	add	w1, w0, 1
	str	w1, [sp, 8]
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	ldrsw	x2, [sp, 12]
	ldr	w2, [x1, x2, lsl 2]
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	sxtw	x0, w0
	str	w2, [x1, x0, lsl 2]
.L69:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	str	wzr, [x0]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	str	wzr, [x0]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	str	wzr, [x0]
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L68:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L70
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w1, [sp, 8]
	str	w1, [x0]
	ldr	w0, [sp, 8]
	str	w0, [sp, 12]
	b	.L71
.L72:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L71:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L72
	add	sp, sp, 16
	ret
	.size	PointPointer, .-PointPointer
	.align	2
	.type	CCO, %function
CCO:
	sub	sp, sp, #16
	str	w0, [sp, 12]
	str	x1, [sp]
	str	w2, [sp, 8]
	ldr	w0, [sp, 8]
	cmp	w0, wzr
	bne	.L74
	ldr	w0, [sp, 12]
	and	w0, w0, 32
	cmp	w0, wzr
	beq	.L75
	ldr	w0, [sp, 12]
	and	w1, w0, -32
	ldr	w0, [sp, 12]
	mvn	w0, w0
	uxtw	x0, w0
	and	x0, x0, 31
	lsl	x0, x0, 2
	ldr	x2, [sp]
	add	x0, x2, x0
	ldr	w0, [x0]
	mvn	w0, w0
	and	w0, w0, 31
	add	w0, w1, w0
	b	.L76
.L75:
	ldr	w0, [sp, 12]
	and	w1, w0, -32
	ldr	w0, [sp, 12]
	and	x0, x0, 31
	lsl	x0, x0, 2
	ldr	x2, [sp]
	add	x0, x2, x0
	ldr	w0, [x0]
	and	w0, w0, 31
	add	w0, w1, w0
	b	.L76
.L74:
	ldr	w0, [sp, 8]
	cmp	w0, 1
	bne	.L77
	ldr	w0, [sp, 12]
	and	w0, w0, 64
	cmp	w0, wzr
	beq	.L78
	ldr	w0, [sp, 12]
	and	w1, w0, -64
	ldr	w0, [sp, 12]
	mvn	w0, w0
	uxtw	x0, w0
	and	x0, x0, 63
	lsl	x0, x0, 2
	ldr	x2, [sp]
	add	x0, x2, x0
	ldr	w0, [x0]
	mvn	w0, w0
	and	w0, w0, 63
	add	w0, w1, w0
	b	.L76
.L78:
	ldr	w0, [sp, 12]
	and	w1, w0, -64
	ldr	w0, [sp, 12]
	and	x0, x0, 63
	lsl	x0, x0, 2
	ldr	x2, [sp]
	add	x0, x2, x0
	ldr	w0, [x0]
	and	w0, w0, 63
	add	w0, w1, w0
	b	.L76
.L77:
	ldr	w0, [sp, 8]
	cmp	w0, 2
	bne	.L79
	ldr	w0, [sp, 12]
	and	w1, w0, -64
	ldr	w0, [sp, 12]
	and	x0, x0, 63
	lsl	x0, x0, 2
	ldr	x2, [sp]
	add	x0, x2, x0
	ldr	w0, [x0]
	and	w0, w0, 63
	add	w0, w1, w0
	b	.L76
.L79:
	mov	w0, 0
.L76:
	add	sp, sp, 16
	ret
	.size	CCO, .-CCO
	.align	2
	.type	CoordinateCorrect, %function
CoordinateCorrect:
	stp	x29, x30, [sp, -192]!
	add	x29, sp, 0
	mov	w0, 64
	str	w0, [x29, 180]
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	ldr	w0, [x0]
	and	w0, w0, 15
	cmp	w0, wzr
	beq	.L80
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	ldr	w0, [x0]
	lsr	w0, w0, 4
	and	w0, w0, 15
	str	w0, [x29, 176]
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	ldr	w0, [x0]
	lsr	w0, w0, 8
	and	w0, w0, 15
	str	w0, [x29, 172]
	add	x0, x29, 128
	adrp	x1, coordinate_correct_coe_x
	add	x1, x1, :lo12:coordinate_correct_coe_x
	str	x1, [x0]
	add	x0, x29, 88
	adrp	x1, coordinate_correct_coe_y
	add	x1, x1, :lo12:coordinate_correct_coe_y
	str	x1, [x0]
	str	wzr, [x29, 188]
	b	.L83
.L84:
	ldr	w0, [x29, 188]
	add	w1, w0, 1
	add	x0, x29, 128
	sxtw	x1, w1
	str	xzr, [x0, x1, lsl 3]
	ldr	w0, [x29, 188]
	add	w1, w0, 1
	add	x0, x29, 88
	sxtw	x1, w1
	str	xzr, [x0, x1, lsl 3]
	ldr	w0, [x29, 188]
	add	w0, w0, 1
	str	w0, [x29, 188]
.L83:
	ldr	w0, [x29, 188]
	cmp	w0, 3
	ble	.L84
	ldr	w0, [x29, 176]
	cmp	w0, 3
	beq	.L85
	ldr	w0, [x29, 172]
	cmp	w0, 3
	bne	.L86
.L85:
	str	wzr, [x29, 188]
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	ldr	w0, [x0]
	lsr	w0, w0, 4
	and	w0, w0, 15
	cmp	w0, 3
	bne	.L87
	ldr	w0, [x29, 188]
	add	w1, w0, 1
	str	w1, [x29, 188]
	sxtw	x0, w0
	lsl	x1, x0, 8
	adrp	x0, multi_group
	add	x0, x0, :lo12:multi_group
	add	x1, x1, x0
	add	x0, x29, 128
	str	x1, [x0, 8]
.L87:
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	ldr	w0, [x0]
	lsr	w0, w0, 8
	and	w0, w0, 15
	cmp	w0, 3
	bne	.L88
	ldr	w0, [x29, 188]
	add	w1, w0, 1
	str	w1, [x29, 188]
	sxtw	x0, w0
	lsl	x1, x0, 8
	adrp	x0, multi_group
	add	x0, x0, :lo12:multi_group
	add	x1, x1, x0
	add	x0, x29, 88
	str	x1, [x0, 8]
	b	.L89
.L88:
	b	.L89
.L86:
	str	wzr, [x29, 188]
	b	.L90
.L91:
	adrp	x0, multi_x_array
	add	x0, x0, :lo12:multi_x_array
	ldrsw	x1, [x29, 188]
	ldr	w0, [x0, x1, lsl 2]
	and	w1, w0, 65535
	add	x2, x29, 56
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0]
	adrp	x0, multi_x_array
	add	x0, x0, :lo12:multi_x_array
	ldrsw	x1, [x29, 188]
	ldr	w0, [x0, x1, lsl 2]
	lsr	w1, w0, 16
	add	x2, x29, 56
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0, 4]
	adrp	x0, multi_y_array
	add	x0, x0, :lo12:multi_y_array
	ldrsw	x1, [x29, 188]
	ldr	w0, [x0, x1, lsl 2]
	and	w1, w0, 65535
	add	x2, x29, 24
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0]
	adrp	x0, multi_y_array
	add	x0, x0, :lo12:multi_y_array
	ldrsw	x1, [x29, 188]
	ldr	w0, [x0, x1, lsl 2]
	lsr	w1, w0, 16
	add	x2, x29, 24
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0, 4]
	ldr	w0, [x29, 188]
	add	w0, w0, 1
	str	w0, [x29, 188]
.L90:
	ldr	w0, [x29, 188]
	cmp	w0, 3
	ble	.L91
	mov	w0, 1
	str	w0, [x29, 184]
	str	wzr, [x29, 188]
	b	.L92
.L94:
	add	x1, x29, 56
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L93
	add	x1, x29, 56
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	cmp	w0, 3
	bhi	.L93
	ldr	w0, [x29, 184]
	add	w1, w0, 1
	str	w1, [x29, 184]
	add	x2, x29, 56
	ldrsw	x1, [x29, 188]
	lsl	x1, x1, 3
	add	x1, x2, x1
	ldr	w1, [x1, 4]
	uxtw	x1, w1
	lsl	x2, x1, 8
	adrp	x1, multi_group
	add	x1, x1, :lo12:multi_group
	add	x2, x2, x1
	add	x1, x29, 128
	sxtw	x0, w0
	str	x2, [x1, x0, lsl 3]
.L93:
	ldr	w0, [x29, 188]
	add	w0, w0, 1
	str	w0, [x29, 188]
.L92:
	ldr	w0, [x29, 188]
	cmp	w0, 3
	ble	.L94
	mov	w0, 1
	str	w0, [x29, 184]
	str	wzr, [x29, 188]
	b	.L95
.L97:
	add	x1, x29, 24
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L96
	add	x1, x29, 24
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	cmp	w0, 3
	bhi	.L96
	ldr	w0, [x29, 184]
	add	w1, w0, 1
	str	w1, [x29, 184]
	add	x2, x29, 24
	ldrsw	x1, [x29, 188]
	lsl	x1, x1, 3
	add	x1, x2, x1
	ldr	w1, [x1, 4]
	uxtw	x1, w1
	lsl	x2, x1, 8
	adrp	x1, multi_group
	add	x1, x1, :lo12:multi_group
	add	x2, x2, x1
	add	x1, x29, 88
	sxtw	x0, w0
	str	x2, [x1, x0, lsl 3]
.L96:
	ldr	w0, [x29, 188]
	add	w0, w0, 1
	str	w0, [x29, 188]
.L95:
	ldr	w0, [x29, 188]
	cmp	w0, 3
	ble	.L97
.L89:
	str	wzr, [x29, 188]
	b	.L98
.L118:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 188]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	bne	.L99
	nop
	b	.L80
.L99:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L101
	b	.L102
.L101:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 180]
	cmp	w1, w0
	bcc	.L103
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 180]
	sub	w0, w1, w0
	cmp	w2, w0
	bhi	.L103
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L104
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w3, w0
	adrp	x0, multi_group+512
	add	x0, x0, :lo12:multi_group+512
	mov	w2, 2
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	b	.L103
.L104:
	ldr	w0, [x29, 176]
	cmp	w0, 3
	bne	.L105
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	and	w0, w0, 64
	cmp	w0, wzr
	beq	.L106
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w3, w0
	add	x0, x29, 128
	ldr	x0, [x0]
	mov	w2, 2
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	b	.L103
.L106:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w3, w0
	add	x0, x29, 128
	ldr	x0, [x0, 8]
	mov	w2, 2
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	b	.L103
.L105:
	str	wzr, [x29, 184]
	b	.L108
.L110:
	ldr	w0, [x29, 184]
	cmp	w0, 3
	bgt	.L109
	ldr	w0, [x29, 184]
	add	w1, w0, 1
	add	x0, x29, 128
	sxtw	x1, w1
	ldr	x0, [x0, x1, lsl 3]
	cmp	x0, xzr
	beq	.L109
	add	x1, x29, 56
	ldrsw	x0, [x29, 184]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L109
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	add	x1, x29, 56
	ldrsw	x0, [x29, 184]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w2, w0
	bcc	.L109
	ldr	w0, [x29, 184]
	add	w0, w0, 1
	str	w0, [x29, 184]
	b	.L108
.L109:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w3, w0
	add	x0, x29, 128
	ldrsw	x1, [x29, 184]
	ldr	x0, [x0, x1, lsl 3]
	ldr	w2, [x29, 176]
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	b	.L103
.L108:
	ldr	w0, [x29, 184]
	cmp	w0, 4
	ble	.L110
.L103:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 180]
	cmp	w1, w0
	bcc	.L102
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 180]
	sub	w0, w1, w0
	cmp	w2, w0
	bhi	.L102
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L111
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	adrp	x0, multi_group+768
	add	x0, x0, :lo12:multi_group+768
	mov	w2, 2
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L102
.L111:
	ldr	w0, [x29, 172]
	cmp	w0, 3
	bne	.L112
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	and	w0, w0, 64
	cmp	w0, wzr
	beq	.L113
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	add	x0, x29, 88
	ldr	x0, [x0]
	mov	w2, 2
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L102
.L113:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	add	x0, x29, 88
	ldr	x0, [x0, 8]
	mov	w2, 2
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L102
.L112:
	str	wzr, [x29, 184]
	b	.L115
.L117:
	ldr	w0, [x29, 184]
	cmp	w0, 3
	bgt	.L116
	ldr	w0, [x29, 184]
	add	w1, w0, 1
	add	x0, x29, 88
	sxtw	x1, w1
	ldr	x0, [x0, x1, lsl 3]
	cmp	x0, xzr
	beq	.L116
	add	x1, x29, 24
	ldrsw	x0, [x29, 184]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L116
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	add	x1, x29, 24
	ldrsw	x0, [x29, 184]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w2, w0
	bcc	.L116
	ldr	w0, [x29, 184]
	add	w0, w0, 1
	str	w0, [x29, 184]
	b	.L115
.L116:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	add	x0, x29, 88
	ldrsw	x1, [x29, 184]
	ldr	x0, [x0, x1, lsl 3]
	ldr	w2, [x29, 172]
	mov	x1, x0
	mov	w0, w3
	bl	CCO
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 188]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L102
.L115:
	ldr	w0, [x29, 184]
	cmp	w0, 4
	ble	.L117
.L102:
	ldr	w0, [x29, 188]
	add	w0, w0, 1
	str	w0, [x29, 188]
.L98:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [x29, 188]
	cmp	w1, w0
	bge	.L80
	ldr	w0, [x29, 188]
	cmp	w0, 9
	ble	.L118
.L80:
	ldp	x29, x30, [sp], 192
	ret
	.size	CoordinateCorrect, .-CoordinateCorrect
	.align	2
	.type	PointPredictOne, %function
PointPredictOne:
	sub	sp, sp, #16
	str	w0, [sp, 12]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1, 8]
	ldr	w1, [sp, 12]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	and	w1, w1, 268374015
	str	w1, [x0]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	and	w1, w1, 127
	strb	w1, [x0, 1]
	add	sp, sp, 16
	ret
	.size	PointPredictOne, .-PointPredictOne
	.align	2
	.type	PointPredictTwo, %function
PointPredictTwo:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	str	w0, [x29, 44]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	lsl	w0, w0, 1
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1, 16]
	ldr	w1, [x29, 44]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	sub	w0, w0, w1
	str	w0, [x29, 60]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	lsl	w0, w0, 1
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1, 16]
	ldr	w1, [x29, 44]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1]
	ubfx	x1, x1, 0, 12
	uxth	w1, w1
	sub	w0, w0, w1
	str	w0, [x29, 56]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x19, x1, x0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldr	w1, [x29, 56]
	ldr	w0, [x29, 60]
	bl	PointRange
	str	w0, [x19]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	orr	w1, w1, -128
	strb	w1, [x0, 1]
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 64
	ret
	.size	PointPredictTwo, .-PointPredictTwo
	.align	2
	.type	PointPredictThree, %function
PointPredictThree:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	str	w0, [x29, 44]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	mov	w0, w1
	lsl	w0, w0, 2
	add	w0, w0, w1
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1, 24]
	ldr	w1, [x29, 44]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	add	w1, w0, w1
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0, 16]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	mov	w0, w2
	lsl	w2, w2, 1
	sub	w0, w0, w2
	lsl	w0, w0, 2
	add	w0, w1, w0
	str	w0, [x29, 60]
	ldr	w0, [x29, 60]
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [x29, 60]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	mov	w0, w1
	lsl	w0, w0, 2
	add	w0, w0, w1
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1, 24]
	ldr	w1, [x29, 44]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1]
	ubfx	x1, x1, 0, 12
	uxth	w1, w1
	add	w1, w0, w1
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0, 16]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	mov	w0, w2
	lsl	w2, w2, 1
	sub	w0, w0, w2
	lsl	w0, w0, 2
	add	w0, w1, w0
	str	w0, [x29, 56]
	ldr	w0, [x29, 56]
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [x29, 56]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x19, x1, x0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldr	w1, [x29, 56]
	ldr	w0, [x29, 60]
	bl	PointRange
	str	w0, [x19]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	orr	w1, w1, -128
	strb	w1, [x0, 1]
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 64
	ret
	.size	PointPredictThree, .-PointPredictThree
	.align	2
	.type	PointPredict, %function
PointPredict:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	wzr, [x29, 28]
	b	.L123
.L131:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L124
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 15
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L125
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 16]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L125
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 16]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L125
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 24]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L125
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L125
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L126
.L125:
	ldr	w0, [x29, 28]
	bl	PointPredictOne
	b	.L127
.L126:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 16]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L127
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 24]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L128
	ldr	w0, [x29, 28]
	bl	PointPredictThree
	b	.L127
.L128:
	ldr	w0, [x29, 28]
	bl	PointPredictTwo
.L127:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1]
	ldrsw	x1, [x29, 28]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	orr	w1, w1, 8192
	str	w1, [x0]
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, pressure_pointer
	add	x1, x1, :lo12:pressure_pointer
	ldr	x2, [x1, 8]
	ldrsw	x1, [x29, 28]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	b	.L129
.L124:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, 268374015
	str	w1, [x0]
.L129:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L130
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1]
	ldrsw	x1, [x29, 28]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	orr	w1, w1, 4096
	str	w1, [x0]
.L130:
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L123:
	ldr	w0, [x29, 28]
	cmp	w0, 9
	ble	.L131
	ldp	x29, x30, [sp], 32
	ret
	.size	PointPredict, .-PointPredict
	.align	2
	.type	PointDistance, %function
PointDistance:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	x1, [sp]
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	ldrb	w0, [x0]
	and	w0, w0, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L133
	ldr	x0, [sp, 8]
	ldrh	w0, [x0, 2]
	str	w0, [sp, 24]
	ldr	x0, [sp]
	ldrh	w0, [x0, 2]
	str	w0, [sp, 20]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w1, w1, w0
	ldr	w2, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w0, w2, w0
	mul	w0, w1, w0
	str	w0, [sp, 28]
	ldr	x0, [sp, 8]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 13
	uxth	w0, w0
	lsl	w1, w0, 6
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	sdiv	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	mul	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	add	w1, w0, 63
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 6
	str	w0, [sp, 24]
	ldr	x0, [sp]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 13
	uxth	w0, w0
	lsl	w1, w0, 6
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	sdiv	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	mul	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	add	w1, w0, 63
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 6
	str	w0, [sp, 20]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w1, w1, w0
	ldr	w2, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w0, w2, w0
	mul	w0, w1, w0
	ldr	w1, [sp, 28]
	add	w0, w1, w0
	str	w0, [sp, 28]
	b	.L134
.L133:
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	ldrb	w0, [x0]
	and	w0, w0, 2
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L135
	ldr	x0, [sp, 8]
	ldrh	w0, [x0, 2]
	lsl	w1, w0, 6
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	sdiv	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	mul	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	add	w1, w0, 63
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 6
	str	w0, [sp, 24]
	ldr	x0, [sp]
	ldrh	w0, [x0, 2]
	lsl	w1, w0, 6
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	sdiv	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	mul	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	add	w1, w0, 63
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 6
	str	w0, [sp, 20]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w1, w1, w0
	ldr	w2, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w0, w2, w0
	mul	w0, w1, w0
	str	w0, [sp, 28]
	ldr	x0, [sp, 8]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 13
	uxth	w0, w0
	str	w0, [sp, 24]
	ldr	x0, [sp]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 13
	uxth	w0, w0
	str	w0, [sp, 20]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w1, w1, w0
	ldr	w2, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w0, w2, w0
	mul	w0, w1, w0
	ldr	w1, [sp, 28]
	add	w0, w1, w0
	str	w0, [sp, 28]
	b	.L134
.L135:
	ldr	x0, [sp, 8]
	ldrh	w0, [x0, 2]
	str	w0, [sp, 24]
	ldr	x0, [sp]
	ldrh	w0, [x0, 2]
	str	w0, [sp, 20]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w1, w1, w0
	ldr	w2, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w0, w2, w0
	mul	w0, w1, w0
	str	w0, [sp, 28]
	ldr	x0, [sp, 8]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 13
	uxth	w0, w0
	str	w0, [sp, 24]
	ldr	x0, [sp]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 13
	uxth	w0, w0
	str	w0, [sp, 20]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w1, w1, w0
	ldr	w2, [sp, 24]
	ldr	w0, [sp, 20]
	sub	w0, w2, w0
	mul	w0, w1, w0
	ldr	w1, [sp, 28]
	add	w0, w1, w0
	str	w0, [sp, 28]
.L134:
	ldr	w0, [sp, 28]
	add	sp, sp, 32
	ret
	.size	PointDistance, .-PointDistance
	.align	2
	.type	DistanceInit, %function
DistanceInit:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	ldr	x0, [sp, 8]
	add	x0, x0, 12
	str	x0, [sp, 16]
	str	wzr, [sp, 28]
	b	.L138
.L139:
	ldr	x0, [sp, 16]
	add	x1, x0, 4
	str	x1, [sp, 16]
	mov	w1, 2147483647
	str	w1, [x0]
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L138:
	ldr	w0, [sp, 28]
	cmp	w0, 99
	ble	.L139
	add	sp, sp, 32
	ret
	.size	DistanceInit, .-DistanceInit
	.align	2
	.type	DistanceMin, %function
DistanceMin:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	ldr	x0, [sp, 8]
	mov	w1, 2147483647
	str	w1, [x0, 8]
	str	wzr, [sp, 24]
	b	.L141
.L145:
	str	wzr, [sp, 28]
	b	.L142
.L144:
	ldr	x2, [sp, 8]
	ldrsw	x3, [sp, 28]
	ldrsw	x1, [sp, 24]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w1, [x0, 12]
	ldr	x0, [sp, 8]
	ldr	w0, [x0, 8]
	cmp	w1, w0
	bcs	.L143
	ldr	w1, [sp, 28]
	ldr	x0, [sp, 8]
	str	w1, [x0]
	ldr	w1, [sp, 24]
	ldr	x0, [sp, 8]
	str	w1, [x0, 4]
	ldr	x2, [sp, 8]
	ldrsw	x3, [sp, 28]
	ldrsw	x1, [sp, 24]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w1, [x0, 12]
	ldr	x0, [sp, 8]
	str	w1, [x0, 8]
.L143:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L142:
	ldr	w0, [sp, 28]
	cmp	w0, 9
	ble	.L144
	ldr	w0, [sp, 24]
	add	w0, w0, 1
	str	w0, [sp, 24]
.L141:
	ldr	w0, [sp, 24]
	cmp	w0, 9
	ble	.L145
	ldr	x0, [sp, 8]
	ldr	w1, [x0, 8]
	mov	w0, 2147483647
	cmp	w1, w0
	bne	.L146
	mov	w0, 0
	b	.L147
.L146:
	mov	w0, 1
.L147:
	add	sp, sp, 32
	ret
	.size	DistanceMin, .-DistanceMin
	.align	2
	.type	DistanceIgnore, %function
DistanceIgnore:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	wzr, [sp, 28]
	b	.L149
.L150:
	ldr	x0, [sp, 8]
	ldr	w0, [x0, 4]
	ldr	x2, [sp, 8]
	ldrsw	x3, [sp, 28]
	uxtw	x1, w0
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	lsl	x0, x0, 2
	add	x0, x2, x0
	mov	w1, 2147483647
	str	w1, [x0, 12]
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L149:
	ldr	w0, [sp, 28]
	cmp	w0, 9
	ble	.L150
	str	wzr, [sp, 24]
	b	.L151
.L152:
	ldr	x0, [sp, 8]
	ldr	w0, [x0]
	ldr	x2, [sp, 8]
	uxtw	x3, w0
	ldrsw	x1, [sp, 24]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	lsl	x0, x0, 2
	add	x0, x2, x0
	mov	w1, 2147483647
	str	w1, [x0, 12]
	ldr	w0, [sp, 24]
	add	w0, w0, 1
	str	w0, [sp, 24]
.L151:
	ldr	w0, [sp, 24]
	cmp	w0, 9
	ble	.L152
	add	sp, sp, 32
	ret
	.size	DistanceIgnore, .-DistanceIgnore
	.align	2
	.type	SpeedGet, %function
SpeedGet:
	sub	sp, sp, #32
	str	w0, [sp, 12]
	mov	w0, 8
	str	w0, [sp, 28]
	b	.L154
.L157:
	ldr	w0, [sp, 28]
	mov	w1, 256
	lsl	w1, w1, w0
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bge	.L155
	b	.L156
.L155:
	ldr	w0, [sp, 28]
	sub	w0, w0, #1
	str	w0, [sp, 28]
.L154:
	ldr	w0, [sp, 28]
	cmp	w0, wzr
	bgt	.L157
.L156:
	ldr	w0, [sp, 28]
	add	sp, sp, 32
	ret
	.size	SpeedGet, .-SpeedGet
	.align	2
	.type	PointId, %function
PointId:
	stp	x29, x30, [sp, -480]!
	add	x29, sp, 0
	add	x0, x29, 56
	bl	DistanceInit
	str	wzr, [x29, 476]
	b	.L160
.L164:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 476]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, -128
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L161
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 476]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L162
.L161:
	adrp	x0, id_first_coe
	add	x0, x0, :lo12:id_first_coe
	ldr	w2, [x0]
	add	x0, x29, 16
	ldrsw	x1, [x29, 476]
	str	w2, [x0, x1, lsl 2]
	b	.L163
.L162:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 476]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 476]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	bl	SpeedGet
	mov	w2, w0
	add	x0, x29, 16
	ldrsw	x1, [x29, 476]
	str	w2, [x0, x1, lsl 2]
.L163:
	ldr	w0, [x29, 476]
	add	w0, w0, 1
	str	w0, [x29, 476]
.L160:
	ldr	w0, [x29, 476]
	cmp	w0, 9
	ble	.L164
	str	wzr, [x29, 476]
	b	.L165
.L170:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 476]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [x0]
	mov	w0, 268374015
	cmp	w1, w0
	bne	.L166
	b	.L167
.L166:
	str	wzr, [x29, 472]
	b	.L168
.L169:
	ldrsw	x0, [x29, 472]
	lsl	x1, x0, 2
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	add	x2, x1, x0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 476]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	mov	w4, w0
	add	x2, x29, 56
	ldrsw	x3, [x29, 476]
	ldrsw	x1, [x29, 472]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	lsl	x0, x0, 2
	add	x0, x2, x0
	str	w4, [x0, 12]
	ldr	w0, [x29, 472]
	add	w0, w0, 1
	str	w0, [x29, 472]
.L168:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [x29, 472]
	cmp	w1, w0
	bge	.L167
	ldr	w0, [x29, 472]
	cmp	w0, 9
	ble	.L169
.L167:
	ldr	w0, [x29, 476]
	add	w0, w0, 1
	str	w0, [x29, 476]
.L165:
	ldr	w0, [x29, 476]
	cmp	w0, 9
	ble	.L170
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L159
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L173
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L174
.L173:
	add	x0, x29, 56
	bl	DistanceMin
	cmp	w0, wzr
	beq	.L175
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 8]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L176
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 8]
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w1, w0
	add	x0, x29, 56
	ldr	w3, [x0, 4]
	adrp	x0, point_now
	add	x2, x0, :lo12:point_now
	uxtw	x0, w3
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w0, w0
	cmp	w1, w0
	beq	.L176
	add	x0, x29, 56
	bl	DistanceIgnore
	b	.L177
.L176:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	add	x1, x29, 56
	ldr	w2, [x1, 4]
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	uxtw	x2, w2
	ldr	w1, [x1, x2, lsl 2]
	str	w1, [x0]
	b	.L178
.L175:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	ldr	w1, [x1]
	str	w1, [x0]
.L178:
	str	wzr, [x29, 476]
	b	.L179
.L180:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 476]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 476]
	add	w0, w0, 1
	str	w0, [x29, 476]
.L179:
	ldr	w0, [x29, 476]
	cmp	w0, 9
	ble	.L180
.L177:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	mov	w1, 1
	str	w1, [x0]
	b	.L181
.L174:
	str	wzr, [x29, 472]
	b	.L182
.L186:
	add	x0, x29, 56
	bl	DistanceMin
	cmp	w0, wzr
	bne	.L183
	b	.L181
.L183:
	add	x0, x29, 56
	ldr	w1, [x0, 8]
	add	x0, x29, 56
	ldr	w2, [x0]
	add	x0, x29, 16
	uxtw	x2, w2
	ldr	w2, [x0, x2, lsl 2]
	adrp	x0, id_speed_coe
	add	x0, x0, :lo12:id_speed_coe
	ldr	w0, [x0]
	mul	w2, w2, w0
	adrp	x0, id_static_coe
	add	x0, x0, :lo12:id_static_coe
	ldr	w0, [x0]
	add	w0, w2, w0
	cmp	w1, w0
	bcc	.L184
	b	.L185
.L184:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	add	x0, x29, 56
	ldr	w0, [x0]
	uxtw	x0, w0
	lsl	x0, x0, 2
	add	x0, x1, x0
	add	x1, x29, 56
	ldr	w2, [x1, 4]
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	uxtw	x2, w2
	ldr	w1, [x1, x2, lsl 2]
	str	w1, [x0]
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x1, [x0]
	add	x0, x29, 56
	ldr	w0, [x0]
	uxtw	x0, w0
	lsl	x0, x0, 2
	add	x0, x1, x0
	add	x1, x29, 56
	ldr	w2, [x1, 4]
	adrp	x1, pressure_now
	add	x1, x1, :lo12:pressure_now
	uxtw	x2, w2
	ldr	w1, [x1, x2, lsl 2]
	str	w1, [x0]
	add	x0, x29, 56
	ldr	w1, [x0, 4]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	uxtw	x1, w1
	str	wzr, [x0, x1, lsl 2]
	add	x0, x29, 56
	bl	DistanceIgnore
.L185:
	ldr	w0, [x29, 472]
	add	w0, w0, 1
	str	w0, [x29, 472]
.L182:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [x29, 472]
	cmp	w1, w0
	bge	.L159
	ldr	w0, [x29, 472]
	cmp	w0, 9
	ble	.L186
	b	.L159
.L181:
.L159:
	ldp	x29, x30, [sp], 480
	ret
	.size	PointId, .-PointId
	.align	2
	.type	ClearLenPP, %function
ClearLenPP:
	sub	sp, sp, #32
	str	w0, [sp, 12]
	str	wzr, [sp, 28]
	b	.L188
.L191:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldrsw	x1, [sp, 28]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L189
	b	.L190
.L189:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L188:
	ldr	w0, [sp, 28]
	cmp	w0, 9
	ble	.L191
.L190:
	ldr	w0, [sp, 28]
	add	sp, sp, 32
	ret
	.size	ClearLenPP, .-ClearLenPP
	.align	2
	.type	PointNewId, %function
PointNewId:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	wzr, [x29, 24]
	b	.L194
.L196:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 24]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w1, w0, 268374015
	mov	w0, 268374015
	cmp	w1, w0
	bne	.L195
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 24]
	lsl	x0, x0, 2
	add	x0, x1, x0
	str	wzr, [x0]
.L195:
	ldr	w0, [x29, 24]
	add	w0, w0, 1
	str	w0, [x29, 24]
.L194:
	ldr	w0, [x29, 24]
	cmp	w0, 9
	ble	.L196
	str	wzr, [x29, 24]
	b	.L197
.L203:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 24]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L198
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [x29, 24]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L199
	b	.L198
.L199:
	mov	w0, 1
	str	w0, [x29, 28]
	b	.L200
.L202:
	ldr	w0, [x29, 28]
	sub	w0, w0, #1
	bl	ClearLenPP
	cmp	w0, 2
	ble	.L201
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	sub	x0, x0, #4
	add	x0, x1, x0
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	ldrsw	x2, [x29, 24]
	ldr	w1, [x1, x2, lsl 2]
	str	w1, [x0]
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	sub	x0, x0, #4
	add	x0, x1, x0
	adrp	x1, pressure_now
	add	x1, x1, :lo12:pressure_now
	ldrsw	x2, [x29, 24]
	ldr	w1, [x1, x2, lsl 2]
	str	w1, [x0]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 24]
	str	wzr, [x0, x1, lsl 2]
	b	.L198
.L201:
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L200:
	ldr	w0, [x29, 28]
	cmp	w0, 10
	ble	.L202
.L198:
	ldr	w0, [x29, 24]
	add	w0, w0, 1
	str	w0, [x29, 24]
.L197:
	ldr	w0, [x29, 24]
	cmp	w0, 9
	ble	.L203
	ldp	x29, x30, [sp], 32
	ret
	.size	PointNewId, .-PointNewId
	.align	2
	.type	PointOrder, %function
PointOrder:
	sub	sp, sp, #16
	str	wzr, [sp, 12]
	b	.L205
.L209:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L206
	b	.L207
.L206:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L208
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L208
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L208
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmp	w0, 1
	bne	.L207
.L208:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	str	wzr, [x0]
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L207:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L205:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L209
	add	sp, sp, 16
	ret
	.size	PointOrder, .-PointOrder
	.align	2
	.type	PointCross, %function
PointCross:
	sub	sp, sp, #16
	str	wzr, [sp, 8]
	b	.L211
.L221:
	ldr	w0, [sp, 8]
	add	w0, w0, 1
	str	w0, [sp, 12]
	b	.L212
.L220:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L213
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L213
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L213
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L214
.L213:
	b	.L215
.L214:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w1, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	cmp	w1, w0
	bcs	.L216
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w1, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0, 8]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	cmp	w1, w0
	bhi	.L217
.L216:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w1, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	cmp	w1, w0
	bls	.L215
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w1, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0, 8]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	cmp	w1, w0
	bcs	.L215
.L217:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	cmp	w2, w0
	bge	.L218
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	cmp	w2, w0
	bgt	.L219
.L218:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	cmp	w2, w0
	ble	.L215
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0, 8]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	cmp	w2, w0
	bge	.L215
.L219:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [sp, 4]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1]
	ldr	w1, [sp, 8]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	strh	w1, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [sp, 4]
	uxth	w1, w1
	strh	w1, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [sp, 4]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 12]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldr	w0, [sp, 8]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldr	w0, [sp, 4]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L215:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L212:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	bls	.L220
	ldr	w0, [sp, 8]
	add	w0, w0, 1
	str	w0, [sp, 8]
.L211:
	ldr	w0, [sp, 8]
	cmp	w0, 9
	bls	.L221
	add	sp, sp, 16
	ret
	.size	PointCross, .-PointCross
	.align	2
	.type	GetPointNum, %function
GetPointNum:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	wzr, [x0]
	str	wzr, [sp, 28]
	b	.L223
.L225:
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L224
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	add	w1, w0, 1
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	w1, [x0]
.L224:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L223:
	ldr	w0, [sp, 28]
	cmp	w0, 9
	ble	.L225
	add	sp, sp, 32
	ret
	.size	GetPointNum, .-GetPointNum
	.align	2
	.type	PointDelay, %function
PointDelay:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	wzr, [x29, 28]
	b	.L227
.L245:
	adrp	x0, report_delay
	add	x0, x0, :lo12:report_delay
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L228
	adrp	x0, delay_key
	add	x0, x0, :lo12:delay_key
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L228
	adrp	x0, point_delay
	add	x0, x0, :lo12:point_delay
	ldrsw	x1, [x29, 28]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L229
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 3]
	orr	w1, w1, 64
	strb	w1, [x0, 3]
	b	.L230
.L229:
	b	.L230
.L228:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L231
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 7, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L231
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L231
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L232
	b	.L230
.L232:
	adrp	x0, delay_key
	add	x0, x0, :lo12:delay_key
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L233
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L233
	adrp	x0, delay_key
	add	x0, x0, :lo12:delay_key
	ldr	w2, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	mov	w1, 10
	cmp	w0, 10
	csel	w0, w0, w1, le
	sub	w1, w0, #1
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	lsr	w0, w2, w0
	uxtb	w0, w0
	and	w0, w0, 7
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strb	wzr, [x0, 1]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strb	wzr, [x0, 2]
	b	.L234
.L233:
	adrp	x0, report_delay
	add	x0, x0, :lo12:report_delay
	ldr	w2, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	mov	w1, 10
	cmp	w0, 10
	csel	w0, w0, w1, le
	sub	w1, w0, #1
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	lsr	w0, w2, w0
	uxtb	w0, w0
	and	w0, w0, 7
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0]
	adrp	x0, report_ahead
	add	x0, x0, :lo12:report_ahead
	ldr	w2, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	mov	w1, 10
	cmp	w0, 10
	csel	w0, w0, w1, le
	sub	w1, w0, #1
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	lsr	w0, w2, w0
	uxtb	w0, w0
	and	w0, w0, 7
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 1]
	adrp	x0, report_delete
	add	x0, x0, :lo12:report_delete
	ldr	w2, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	mov	w1, 10
	cmp	w0, 10
	csel	w0, w0, w1, le
	sub	w1, w0, #1
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	lsr	w0, w2, w0
	uxtb	w0, w0
	and	w0, w0, 7
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 2]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	adrp	x0, point_delay
	add	x2, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrb	w0, [x0]
	cmp	w1, w0
	bls	.L235
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w2, [x0]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 1]
.L235:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0]
	adrp	x0, point_delay
	add	x2, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrb	w0, [x0, 1]
	sub	w0, w1, w0
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 1]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 2]
	adrp	x0, point_delay
	add	x2, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrb	w0, [x0, 1]
	cmp	w1, w0
	bls	.L236
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w2, [x0, 1]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 2]
.L236:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	adrp	x0, point_delay
	add	x2, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrb	w0, [x0, 2]
	sub	w0, w1, w0
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 2]
.L234:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 3]
	orr	w1, w1, -128
	strb	w1, [x0, 3]
.L231:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L237
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 3]
	and	w1, w1, 127
	strb	w1, [x0, 3]
.L237:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L238
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 7, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L238
	str	wzr, [x29, 24]
	b	.L239
.L241:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldrsw	x1, [x29, 24]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L240
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldrsw	x1, [x29, 24]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L240
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldrsw	x1, [x29, 24]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 64
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L240
	ldr	w0, [x29, 24]
	add	w0, w0, 1
	str	w0, [x29, 24]
.L239:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0]
	mov	w1, w0
	ldr	w0, [x29, 24]
	cmp	w1, w0
	bge	.L241
.L240:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0]
	mov	w1, w0
	ldr	w0, [x29, 24]
	cmp	w1, w0
	blt	.L242
	b	.L230
.L242:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 3]
	orr	w1, w1, 64
	strb	w1, [x0, 3]
.L238:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 2]
	mov	w1, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L243
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 3]
	and	w1, w1, -65
	strb	w1, [x0, 3]
	b	.L230
.L243:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L244
	b	.L230
.L244:
	adrp	x0, report_delete
	add	x0, x0, :lo12:report_delete
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L230
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	cmp	w0, wzr
	beq	.L230
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	sub	w1, w0, #1
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	cmp	w0, 8
	bhi	.L230
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	sub	w0, w0, #1
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 1]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 2]
	cmp	w0, wzr
	beq	.L230
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 2]
	sub	w0, w0, #1
	uxtb	w2, w0
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strb	w1, [x0, 2]
.L230:
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L227:
	ldr	w0, [x29, 28]
	cmp	w0, 9
	ble	.L245
	ldp	x29, x30, [sp], 32
	ret
	.size	PointDelay, .-PointDelay
	.align	2
	.global	PointMOne
	.type	PointMOne, %function
PointMOne:
	sub	sp, sp, #16
	str	w0, [sp, 12]
	str	w1, [sp, 8]
	ldr	w1, [sp, 8]
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bcc	.L247
	mov	w0, 0
	b	.L248
.L247:
	adrp	x0, edge_start
	add	x0, x0, :lo12:edge_start
	ldr	w0, [x0]
	ldr	w1, [sp, 8]
	cmp	w1, w0
	bcs	.L249
	ldr	w1, [sp, 8]
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	lsr	w1, w0, 1
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bcs	.L249
	mov	w0, 1
	b	.L248
.L249:
	adrp	x0, edge_start
	add	x0, x0, :lo12:edge_start
	ldr	w0, [x0]
	lsl	w1, w0, 1
	ldr	w0, [sp, 8]
	cmp	w1, w0
	bls	.L250
	ldr	w0, [sp, 8]
	lsl	w1, w0, 1
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bcs	.L250
	mov	w0, 1
	b	.L248
.L250:
	mov	w0, 0
.L248:
	add	sp, sp, 16
	ret
	.size	PointMOne, .-PointMOne
	.align	2
	.global	PointMenu
	.type	PointMenu, %function
PointMenu:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	adrp	x0, edge_start
	add	x0, x0, :lo12:edge_start
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L252
	b	.L251
.L252:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L254
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 8]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L254
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 16]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L255
.L254:
	b	.L251
.L255:
	mov	w0, 1
	str	w0, [x29, 28]
	b	.L256
.L258:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L257
	b	.L251
.L257:
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L256:
	ldr	w0, [x29, 28]
	cmp	w0, 9
	ble	.L258
	adrp	x0, point_delay
	add	x0, x0, :lo12:point_delay
	ldrb	w0, [x0]
	cmp	w0, wzr
	beq	.L259
	adrp	x0, point_delay
	add	x0, x0, :lo12:point_delay
	ldrb	w0, [x0, 1]
	cmp	w0, wzr
	bne	.L260
.L259:
	b	.L251
.L260:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 8]
	add	x0, x0, 4
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	mov	w2, w0
	adrp	x0, shake_min
	add	x0, x0, :lo12:shake_min
	ldr	w0, [x0]
	mov	w1, 36
	cmp	w0, 36
	csel	w0, w0, w1, cs
	cmp	w2, w0
	bcs	.L261
	b	.L251
.L261:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 8]
	ldrh	w0, [x0, 2]
	mov	w1, w0
	mov	w0, w2
	bl	PointMOne
	cmp	w0, wzr
	beq	.L262
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0, 8]
	mov	w1, 1
	strh	w1, [x0, 2]
.L262:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0, 8]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	mov	w0, w2
	bl	PointMOne
	cmp	w0, wzr
	beq	.L263
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrh	w0, [x1]
	mov	w2, 1
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L263:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x1, [x1]
	ldrh	w1, [x1, 2]
	sub	w2, w0, w1
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x1, [x1, 8]
	ldrh	w1, [x1, 2]
	sub	w0, w0, w1
	mov	w1, w0
	mov	w0, w2
	bl	PointMOne
	cmp	w0, wzr
	beq	.L264
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0, 8]
	adrp	x1, drv_num_nokey
	add	x1, x1, :lo12:drv_num_nokey
	ldr	w1, [x1]
	uxth	w1, w1
	lsl	w1, w1, 6
	uxth	w1, w1
	sub	w1, w1, #1
	uxth	w1, w1
	strh	w1, [x0, 2]
.L264:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x1, [x1]
	ldrh	w1, [x1]
	ubfx	x1, x1, 0, 12
	uxth	w1, w1
	sub	w2, w0, w1
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x1, [x1, 8]
	ldrh	w1, [x1]
	ubfx	x1, x1, 0, 12
	uxth	w1, w1
	sub	w0, w0, w1
	mov	w1, w0
	mov	w0, w2
	bl	PointMOne
	cmp	w0, wzr
	beq	.L251
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	uxth	w0, w0
	lsl	w0, w0, 6
	uxth	w0, w0
	sub	w0, w0, #1
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L251:
	ldp	x29, x30, [sp], 32
	ret
	.size	PointMenu, .-PointMenu
	.align	2
	.type	FilterOne, %function
FilterOne:
	sub	sp, sp, #48
	str	w0, [sp, 28]
	str	x1, [sp, 16]
	str	x2, [sp, 8]
	str	w3, [sp, 24]
	str	wzr, [sp, 40]
	str	wzr, [sp, 36]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [sp, 28]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L266
	b	.L265
.L266:
	ldr	w0, [sp, 24]
	cmp	w0, wzr
	bgt	.L268
	b	.L265
.L268:
	str	wzr, [sp, 44]
	b	.L269
.L270:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [sp, 44]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	mul	w1, w2, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x2, [sp, 44]
	ldr	x2, [x0, x2, lsl 3]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	mov	w3, w0
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	ldr	x2, [sp, 16]
	add	x0, x2, x0
	ldr	w0, [x0]
	mul	w0, w3, w0
	add	w0, w1, w0
	ldr	w1, [sp, 40]
	add	w0, w1, w0
	str	w0, [sp, 40]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [sp, 44]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	mul	w1, w2, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x2, [sp, 44]
	ldr	x2, [x0, x2, lsl 3]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldrsw	x0, [sp, 44]
	lsl	x0, x0, 2
	ldr	x2, [sp, 16]
	add	x0, x2, x0
	ldr	w0, [x0]
	mul	w0, w3, w0
	add	w0, w1, w0
	ldr	w1, [sp, 36]
	add	w0, w1, w0
	str	w0, [sp, 36]
	ldr	w0, [sp, 44]
	add	w0, w0, 1
	str	w0, [sp, 44]
.L269:
	ldr	w0, [sp, 44]
	cmp	w0, 7
	ble	.L270
	ldr	w0, [sp, 24]
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	mov	w1, w0
	ldr	w0, [sp, 40]
	add	w1, w1, w0
	ldr	w0, [sp, 24]
	sdiv	w0, w1, w0
	str	w0, [sp, 40]
	ldr	w0, [sp, 24]
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	mov	w1, w0
	ldr	w0, [sp, 36]
	add	w1, w1, w0
	ldr	w0, [sp, 24]
	sdiv	w0, w1, w0
	str	w0, [sp, 36]
	ldr	w0, [sp, 40]
	cmp	w0, wzr
	bge	.L271
	str	wzr, [sp, 40]
.L271:
	ldr	w1, [sp, 40]
	mov	w0, 65535
	cmp	w1, w0
	ble	.L272
	mov	w0, 65535
	str	w0, [sp, 40]
.L272:
	ldr	w0, [sp, 36]
	cmp	w0, wzr
	bge	.L273
	str	wzr, [sp, 36]
.L273:
	ldr	w0, [sp, 36]
	cmp	w0, 4095
	ble	.L274
	mov	w0, 4095
	str	w0, [sp, 36]
.L274:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [sp, 40]
	uxth	w1, w1
	strh	w1, [x0, 2]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldr	w0, [sp, 36]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L265:
	add	sp, sp, 48
	ret
	.size	FilterOne, .-FilterOne
	.align	2
	.type	FilterSpeed, %function
FilterSpeed:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	str	w0, [x29, 44]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 44]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	bl	Sqrt
	mov	w19, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 44]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 16]
	ldrsw	x0, [x29, 44]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	bl	Sqrt
	add	w0, w19, w0
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FilterSpeed, .-FilterSpeed
	.align	2
	.type	MedianSpeedOver, %function
MedianSpeedOver:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	w0, [x29, 28]
	str	w1, [x29, 24]
	str	wzr, [x29, 40]
	ldr	w0, [x29, 24]
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	sub	w0, w0, #1
	str	w0, [x29, 24]
	ldr	w0, [x29, 24]
	cmp	w0, wzr
	blt	.L278
	ldr	w0, [x29, 24]
	cmp	w0, 3
	ble	.L279
.L278:
	mov	w0, 1
	b	.L280
.L279:
	adrp	x0, median_dis
	add	x1, x0, :lo12:median_dis
	ldrsw	x0, [x29, 24]
	ldrb	w0, [x1, x0]
	mov	w2, w0
	adrp	x0, median_dis
	add	x1, x0, :lo12:median_dis
	ldrsw	x0, [x29, 24]
	ldrb	w0, [x1, x0]
	mul	w0, w2, w0
	str	w0, [x29, 36]
	str	wzr, [x29, 44]
	b	.L281
.L284:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 44]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x2, x1, x0
	ldr	w0, [x29, 44]
	add	w1, w0, 1
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	mov	w1, w0
	ldr	w0, [x29, 36]
	cmp	w1, w0
	bls	.L282
	ldr	w0, [x29, 40]
	add	w0, w0, 1
	str	w0, [x29, 40]
.L282:
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L281:
	ldr	w1, [x29, 44]
	ldr	w0, [x29, 24]
	cmp	w1, w0
	bgt	.L283
	ldr	w0, [x29, 44]
	cmp	w0, 29
	ble	.L284
.L283:
	ldr	w0, [x29, 40]
	cmp	w0, 1
	ble	.L285
	mov	w0, 1
	b	.L280
.L285:
	mov	w0, 0
.L280:
	ldp	x29, x30, [sp], 48
	ret
	.size	MedianSpeedOver, .-MedianSpeedOver
	.align	2
	.type	PointMedian, %function
PointMedian:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	str	wzr, [x29, 108]
	b	.L287
.L297:
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 108]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 2
	bgt	.L288
	mov	w0, 3
	str	w0, [x29, 100]
	b	.L289
.L288:
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 108]
	ldr	w0, [x0, x1, lsl 2]
	add	w0, w0, 2
	str	w0, [x29, 100]
.L289:
	ldr	w0, [x29, 100]
	cmp	w0, 9
	ble	.L290
	mov	w0, 9
	str	w0, [x29, 100]
.L290:
	ldr	w0, [x29, 100]
	orr	w0, w0, 1
	str	w0, [x29, 100]
	b	.L291
.L296:
	ldr	w1, [x29, 100]
	ldr	w0, [x29, 108]
	bl	MedianSpeedOver
	cmp	w0, wzr
	beq	.L292
	ldr	w0, [x29, 100]
	sub	w0, w0, #2
	str	w0, [x29, 100]
	b	.L291
.L292:
	str	wzr, [x29, 104]
	b	.L293
.L294:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 104]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 108]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	add	x0, x29, 56
	ldrsw	x1, [x29, 104]
	str	w2, [x0, x1, lsl 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 104]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 108]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	add	x0, x29, 16
	ldrsw	x1, [x29, 104]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 104]
	add	w0, w0, 1
	str	w0, [x29, 104]
.L293:
	ldr	w1, [x29, 104]
	ldr	w0, [x29, 100]
	cmp	w1, w0
	blt	.L294
	add	x0, x29, 56
	ldr	w1, [x29, 100]
	bl	SortBubble
	add	x0, x29, 16
	ldr	w1, [x29, 100]
	bl	SortBubble
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 108]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldr	w0, [x29, 100]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	mov	w2, w0
	add	x0, x29, 56
	sxtw	x2, w2
	ldr	w0, [x0, x2, lsl 2]
	uxth	w0, w0
	strh	w0, [x1, 2]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 108]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldr	w0, [x29, 100]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	mov	w2, w0
	add	x0, x29, 16
	sxtw	x2, w2
	ldr	w0, [x0, x2, lsl 2]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L295
.L291:
	ldr	w0, [x29, 100]
	cmp	w0, 2
	bgt	.L296
.L295:
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 108]
	ldr	w2, [x29, 100]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 108]
	add	w0, w0, 1
	str	w0, [x29, 108]
.L287:
	ldr	w0, [x29, 108]
	cmp	w0, 9
	ble	.L297
	ldp	x29, x30, [sp], 112
	ret
	.size	PointMedian, .-PointMedian
	.align	2
	.type	PointFilter, %function
PointFilter:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	str	wzr, [x29, 124]
	b	.L299
.L300:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L299:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L300
	str	wzr, [x29, 124]
	b	.L301
.L307:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L302
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L302
	mov	w0, 1
	str	w0, [x29, 120]
	b	.L303
.L304:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [x29, 120]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L303:
	ldr	w0, [x29, 120]
	cmp	w0, 9
	ble	.L304
	mov	w0, 1
	str	w0, [x29, 120]
	b	.L305
.L306:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 120]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L305:
	ldr	w0, [x29, 120]
	cmp	w0, 9
	ble	.L306
.L302:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L301:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L307
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmp	w0, wzr
	blt	.L308
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmp	w0, 1
	ble	.L298
.L308:
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmp	w0, 1
	ble	.L310
	str	wzr, [x29, 124]
	b	.L311
.L314:
	ldr	w0, [x29, 124]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	mov	w1, w0
	adrp	x0, filter_coe
	add	x0, x0, :lo12:filter_coe
	sxtw	x1, w1
	ldr	w2, [x0, x1, lsl 2]
	ldr	w1, [x29, 124]
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	add	x0, x29, 56
	ldrsw	x1, [x29, 124]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 124]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	add	w1, w0, 2
	adrp	x0, filter_coe
	add	x0, x0, :lo12:filter_coe
	sxtw	x1, w1
	ldr	w2, [x0, x1, lsl 2]
	ldr	w1, [x29, 124]
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	add	x0, x29, 24
	ldrsw	x1, [x29, 124]
	str	w2, [x0, x1, lsl 2]
	add	x0, x29, 56
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 127
	ble	.L312
	add	x0, x29, 56
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	orr	w0, w0, -256
	mov	w2, w0
	add	x0, x29, 56
	ldrsw	x1, [x29, 124]
	str	w2, [x0, x1, lsl 2]
.L312:
	add	x0, x29, 24
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 127
	ble	.L313
	add	x0, x29, 24
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	orr	w0, w0, -256
	mov	w2, w0
	add	x0, x29, 24
	ldrsw	x1, [x29, 124]
	str	w2, [x0, x1, lsl 2]
.L313:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L311:
	ldr	w0, [x29, 124]
	cmp	w0, 7
	ble	.L314
	str	wzr, [x29, 124]
	b	.L315
.L316:
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w2, [x0]
	add	x1, x29, 24
	add	x0, x29, 56
	mov	w3, w2
	mov	x2, x1
	mov	x1, x0
	ldr	w0, [x29, 124]
	bl	FilterOne
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L315:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L316
	b	.L298
.L310:
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmn	w0, #1
	bne	.L318
	bl	PointMedian
	b	.L298
.L318:
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	cmp	w0, wzr
	bge	.L298
	str	wzr, [x29, 124]
	b	.L319
.L320:
	ldr	w0, [x29, 124]
	add	w3, w0, 1
	adrp	x0, median_dis
	add	x1, x0, :lo12:median_dis
	ldrsw	x0, [x29, 124]
	ldrb	w0, [x1, x0]
	mov	w2, w0
	add	x0, x29, 88
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L319:
	ldr	w0, [x29, 124]
	cmp	w0, 3
	ble	.L320
	adrp	x0, median_dis
	add	x0, x0, :lo12:median_dis
	ldrb	w0, [x0]
	lsl	w0, w0, 1
	adrp	x1, median_dis
	add	x1, x1, :lo12:median_dis
	ldrb	w1, [x1, 1]
	sub	w1, w0, w1
	add	x0, x29, 88
	str	w1, [x0]
	adrp	x0, median_dis
	add	x0, x0, :lo12:median_dis
	ldrb	w0, [x0, 3]
	lsr	w0, w0, 1
	uxtb	w0, w0
	mov	w1, w0
	add	x0, x29, 88
	str	w1, [x0, 20]
	str	wzr, [x29, 124]
	b	.L321
.L326:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L322
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	str	wzr, [x0, x1, lsl 2]
	b	.L323
.L322:
	ldr	w0, [x29, 124]
	bl	FilterSpeed
	str	w0, [x29, 116]
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	ble	.L324
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	sub	w1, w0, #1
	add	x0, x29, 88
	sxtw	x1, w1
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 116]
	cmp	w1, w0
	bge	.L324
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	sub	w2, w0, #1
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	str	w2, [x0, x1, lsl 2]
	b	.L325
.L324:
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 2
	bgt	.L325
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	add	w1, w0, 3
	add	x0, x29, 88
	sxtw	x1, w1
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 116]
	cmp	w1, w0
	ble	.L325
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	add	w2, w0, 1
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	str	w2, [x0, x1, lsl 2]
.L325:
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	sxtw	x0, w0
	lsl	x1, x0, 5
	adrp	x0, ps_coe
	add	x0, x0, :lo12:ps_coe
	add	x4, x1, x0
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	sxtw	x0, w0
	lsl	x1, x0, 5
	adrp	x0, pr_coe
	add	x0, x0, :lo12:pr_coe
	add	x1, x1, x0
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	ldr	w0, [x0]
	neg	w0, w0
	mov	w3, w0
	mov	x2, x1
	mov	x1, x4
	ldr	w0, [x29, 124]
	bl	FilterOne
.L323:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L321:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L326
.L298:
	ldp	x29, x30, [sp], 128
	ret
	.size	PointFilter, .-PointFilter
	.align	2
	.type	KeyMap, %function
KeyMap:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	x1, [sp]
	adrp	x0, key_range_array
	add	x0, x0, :lo12:key_range_array
	str	x0, [sp, 16]
	str	wzr, [sp, 28]
	b	.L328
.L331:
	ldr	x0, [sp, 8]
	ldr	w0, [x0]
	mov	w2, w0
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0]
	lsr	w0, w0, 16
	cmp	w2, w0
	bcc	.L329
	ldr	x0, [sp, 8]
	ldr	w0, [x0]
	mov	w2, w0
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0]
	and	w0, w0, 65535
	cmp	w2, w0
	bhi	.L329
	ldr	x0, [sp]
	ldr	w0, [x0]
	mov	w2, w0
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0, 4]
	lsr	w0, w0, 16
	cmp	w2, w0
	bcc	.L329
	ldr	x0, [sp]
	ldr	w0, [x0]
	mov	w2, w0
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0, 4]
	and	w0, w0, 65535
	cmp	w2, w0
	bhi	.L329
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0, 8]
	lsr	w0, w0, 16
	mov	w1, w0
	ldr	x0, [sp]
	str	w1, [x0]
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0, 8]
	and	w1, w0, 65535
	ldr	x0, [sp, 8]
	str	w1, [x0]
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	mov	x1, x0
	ldr	x0, [sp, 16]
	add	x0, x0, x1
	ldr	w0, [x0, 8]
	b	.L330
.L329:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L328:
	ldr	w0, [sp, 28]
	cmp	w0, 7
	ble	.L331
	mov	w0, 0
.L330:
	add	sp, sp, 32
	ret
	.size	KeyMap, .-KeyMap
	.align	2
	.type	ScreenResolution, %function
ScreenResolution:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x0, [x29, 24]
	ldr	x0, [x29, 24]
	ldrh	w0, [x0, 2]
	str	w0, [x29, 44]
	ldr	x0, [x29, 24]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [x29, 40]
	ldr	x0, [x29, 24]
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L333
	ldr	w1, [x29, 40]
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldr	w0, [x0, 4]
	sub	w1, w1, w0
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldr	w0, [x0]
	mul	w0, w1, w0
	add	w0, w0, 2048
	add	w1, w0, 4095
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 12
	str	w0, [x29, 40]
	ldr	w1, [x29, 44]
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldr	w0, [x0, 4]
	sub	w1, w1, w0
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldr	w0, [x0]
	mul	w0, w1, w0
	add	w0, w0, 2048
	add	w1, w0, 4095
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 12
	str	w0, [x29, 44]
.L333:
	ldr	w1, [x29, 40]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	str	w0, [x29, 40]
	ldr	w1, [x29, 44]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	str	w0, [x29, 44]
	ldr	x0, [x29, 24]
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L334
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	ldrb	w0, [x0]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L335
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L336
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	beq	.L337
.L336:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0]
	ldr	w0, [x29, 40]
	cmp	w1, w0
	ble	.L338
	mov	w0, 0
	b	.L354
.L338:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0, 4]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	bgt	.L340
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w1, [x0]
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w0, [x0, 4]
	sub	w1, w1, w0
	ldr	w0, [x29, 40]
	cmp	w1, w0
	bge	.L340
	mov	w0, 0
	b	.L354
.L340:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0, 4]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	blt	.L337
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0, 4]
	ldr	w0, [x29, 40]
	cmp	w1, w0
	bge	.L337
	mov	w0, 0
	b	.L354
.L337:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L341
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	beq	.L335
.L341:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0]
	ldr	w0, [x29, 44]
	cmp	w1, w0
	ble	.L342
	mov	w0, 0
	b	.L354
.L342:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0, 4]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	bgt	.L343
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w1, [x0]
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w0, [x0, 4]
	sub	w1, w1, w0
	ldr	w0, [x29, 44]
	cmp	w1, w0
	bge	.L343
	mov	w0, 0
	b	.L354
.L343:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0, 4]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	blt	.L335
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0, 4]
	ldr	w0, [x29, 44]
	cmp	w1, w0
	bge	.L335
	mov	w0, 0
	b	.L354
.L335:
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	ldr	w0, [x0, 8]
	mov	w1, w0
	ldr	w0, [x29, 40]
	cmp	w1, w0
	blt	.L344
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	ldr	w0, [x0, 8]
	add	w0, w0, 1
	str	w0, [x29, 40]
.L344:
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	adrp	x1, edge_cut
	add	x1, x1, :lo12:edge_cut
	ldr	w1, [x1, 12]
	sub	w1, w0, w1
	ldr	w0, [x29, 40]
	cmp	w1, w0
	bgt	.L345
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	adrp	x1, edge_cut
	add	x1, x1, :lo12:edge_cut
	ldr	w1, [x1, 12]
	sub	w0, w0, w1
	sub	w0, w0, #1
	str	w0, [x29, 40]
.L345:
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [x29, 44]
	cmp	w1, w0
	blt	.L346
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	ldr	w0, [x0]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L346:
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	adrp	x1, edge_cut
	add	x1, x1, :lo12:edge_cut
	ldr	w1, [x1, 4]
	sub	w1, w0, w1
	ldr	w0, [x29, 44]
	cmp	w1, w0
	bgt	.L347
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	adrp	x1, edge_cut
	add	x1, x1, :lo12:edge_cut
	ldr	w1, [x1, 4]
	sub	w0, w0, w1
	sub	w0, w0, #1
	str	w0, [x29, 44]
.L347:
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	ldrb	w0, [x0]
	and	w0, w0, 2
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L348
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w1, [x0]
	ldr	w0, [x29, 40]
	sub	w0, w1, w0
	str	w0, [x29, 40]
.L348:
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	ldrb	w0, [x0]
	and	w0, w0, 4
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L349
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w1, [x0]
	ldr	w0, [x29, 44]
	sub	w0, w1, w0
	str	w0, [x29, 44]
.L349:
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	ldrb	w0, [x0]
	and	w0, w0, 8
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L351
	ldr	w1, [x29, 40]
	ldr	w0, [x29, 44]
	eor	w0, w1, w0
	str	w0, [x29, 40]
	ldr	w1, [x29, 44]
	ldr	w0, [x29, 40]
	eor	w0, w1, w0
	str	w0, [x29, 44]
	ldr	w1, [x29, 40]
	ldr	w0, [x29, 44]
	eor	w0, w1, w0
	str	w0, [x29, 40]
	b	.L351
.L334:
	ldr	w0, [x29, 40]
	cmp	w0, wzr
	bge	.L352
	str	wzr, [x29, 40]
.L352:
	ldr	w0, [x29, 44]
	cmp	w0, wzr
	bge	.L353
	str	wzr, [x29, 44]
.L353:
	adrp	x0, key_map_able
	add	x0, x0, :lo12:key_map_able
	ldr	w0, [x0]
	and	w0, w0, 1
	cmp	w0, wzr
	beq	.L351
	add	x1, x29, 40
	add	x0, x29, 44
	bl	KeyMap
	cmp	w0, wzr
	bne	.L351
	mov	w0, 0
	b	.L354
.L351:
	ldr	w0, [x29, 40]
	lsl	w0, w0, 16
	and	w1, w0, 268369920
	ldr	w0, [x29, 44]
	and	w0, w0, 65535
	add	w0, w1, w0
.L354:
	ldp	x29, x30, [sp], 48
	ret
	.size	ScreenResolution, .-ScreenResolution
	.align	2
	.type	PointReport, %function
PointReport:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	str	x0, [x29, 24]
	str	wzr, [x29, 72]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w1, [x0]
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	ldr	w0, [x0]
	cmp	w1, w0
	ble	.L356
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	ldrb	w0, [x0]
	and	w0, w0, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L356
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	wzr, [x0]
	ldr	x0, [x29, 24]
	str	wzr, [x0, 120]
	b	.L355
.L356:
	str	wzr, [x29, 76]
	b	.L358
.L359:
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L358:
	ldr	w0, [x29, 76]
	cmp	w0, 9
	ble	.L359
	str	wzr, [x29, 72]
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	ldrb	w0, [x0, 3]
	and	w0, w0, 64
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L360
	str	wzr, [x29, 76]
	b	.L361
.L367:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L362
	b	.L363
.L362:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	cmp	w0, 8
	bls	.L364
	b	.L363
.L364:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	add	w1, w0, 1
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 64
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L365
	b	.L363
.L365:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L363
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	orr	w1, w1, 64
	strb	w1, [x0, 1]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	bl	ScreenResolution
	mov	w2, w0
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L363
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	ldr	w1, [x29, 76]
	add	w1, w1, 1
	lsl	w1, w1, 28
	orr	w2, w0, w1
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L363:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L361:
	ldr	w0, [x29, 76]
	cmp	w0, 9
	bgt	.L366
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	ldr	w0, [x0]
	ldr	w1, [x29, 72]
	cmp	w1, w0
	blt	.L367
.L366:
	str	wzr, [x29, 76]
	b	.L368
.L374:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L369
	b	.L370
.L369:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	cmp	w0, 9
	bls	.L371
	b	.L370
.L371:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L372
	b	.L370
.L372:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 64
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L370
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w1, [x0, 1]
	orr	w1, w1, 64
	strb	w1, [x0, 1]
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	bl	ScreenResolution
	mov	w2, w0
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L370
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	ldr	w1, [x29, 76]
	add	w1, w1, 1
	lsl	w1, w1, 28
	orr	w2, w0, w1
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L370:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L368:
	ldr	w0, [x29, 76]
	cmp	w0, 9
	bgt	.L373
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	ldr	w0, [x0]
	ldr	w1, [x29, 72]
	cmp	w1, w0
	blt	.L374
	b	.L373
.L360:
	str	wzr, [x29, 72]
	str	wzr, [x29, 76]
	b	.L375
.L379:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L376
	b	.L377
.L376:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	cmp	w0, 9
	bls	.L378
	b	.L377
.L378:
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	mov	w1, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x0, x1, x0
	bl	ScreenResolution
	mov	w2, w0
	add	x0, x29, 32
	ldrsw	x1, [x29, 72]
	str	w2, [x0, x1, lsl 2]
	add	x0, x29, 32
	ldrsw	x1, [x29, 72]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L377
	ldr	w0, [x29, 72]
	add	w1, w0, 1
	str	w1, [x29, 72]
	add	x1, x29, 32
	sxtw	x2, w0
	ldr	w1, [x1, x2, lsl 2]
	ldr	w2, [x29, 76]
	add	w2, w2, 1
	lsl	w2, w2, 28
	orr	w2, w1, w2
	add	x1, x29, 32
	sxtw	x0, w0
	str	w2, [x1, x0, lsl 2]
.L377:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L375:
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	ldr	w0, [x0]
	ldr	w1, [x29, 76]
	cmp	w1, w0
	bge	.L373
	ldr	w0, [x29, 76]
	cmp	w0, 9
	ble	.L379
.L373:
	str	wzr, [x29, 72]
	str	wzr, [x29, 76]
	b	.L380
.L383:
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	bne	.L381
	b	.L382
.L381:
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w2, [x0, x1, lsl 2]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 72]
	str	w2, [x0, x1, lsl 2]
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	lsr	w0, w0, 16
	and	w2, w0, 4095
	ldr	x0, [x29, 24]
	ldrsw	x1, [x29, 72]
	str	w2, [x0, x1, lsl 2]
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	and	w1, w0, 4095
	ldr	x2, [x29, 24]
	ldrsw	x0, [x29, 72]
	add	x0, x0, 8
	lsl	x0, x0, 2
	add	x0, x2, x0
	str	w1, [x0, 8]
	add	x0, x29, 32
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	lsr	w0, w0, 28
	mov	w2, w0
	ldr	x0, [x29, 24]
	ldrsw	x1, [x29, 72]
	add	x1, x1, 20
	str	w2, [x0, x1, lsl 2]
	adrp	x0, pressure_report
	add	x0, x0, :lo12:pressure_report
	ldrsw	x1, [x29, 76]
	ldr	w2, [x0, x1, lsl 2]
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [x29, 72]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L382:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L380:
	ldr	w0, [x29, 76]
	cmp	w0, 9
	ble	.L383
	ldr	w0, [x29, 72]
	str	w0, [x29, 76]
	b	.L384
.L385:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L384:
	ldr	w0, [x29, 76]
	cmp	w0, 9
	ble	.L385
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w1, [x29, 72]
	str	w1, [x0]
	ldr	x0, [x29, 24]
	ldr	w1, [x29, 72]
	str	w1, [x0, 120]
.L355:
	ldp	x29, x30, [sp], 80
	ret
	.size	PointReport, .-PointReport
	.align	2
	.type	PointEdge, %function
PointEdge:
	sub	sp, sp, #160
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L387
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L388
.L387:
	b	.L386
.L388:
	str	wzr, [sp, 152]
	str	wzr, [sp, 156]
	b	.L390
.L394:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L391
	adrp	x0, stretch_active
	add	x0, x0, :lo12:stretch_active
	ldrsw	x1, [sp, 156]
	ldr	w0, [x0, x1, lsl 2]
	mov	w2, w0
	mov	x0, sp
	ldrsw	x1, [sp, 156]
	str	w2, [x0, x1, lsl 2]
	b	.L392
.L391:
	adrp	x0, stretch_array
	add	x0, x0, :lo12:stretch_array
	ldrsw	x1, [sp, 156]
	ldr	w0, [x0, x1, lsl 2]
	mov	w2, w0
	mov	x0, sp
	ldrsw	x1, [sp, 156]
	str	w2, [x0, x1, lsl 2]
.L392:
	mov	x0, sp
	ldrsw	x1, [sp, 156]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L393
	ldr	w0, [sp, 152]
	add	w0, w0, 1
	str	w0, [sp, 152]
.L393:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L390:
	ldr	w0, [sp, 156]
	cmp	w0, 31
	ble	.L394
	ldr	w0, [sp, 152]
	cmp	w0, wzr
	bne	.L395
	b	.L386
.L395:
	mov	x0, sp
	str	x0, [sp, 136]
	str	wzr, [sp, 156]
	b	.L396
.L407:
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	ldrb	w0, [x0]
	and	w0, w0, 4
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L397
	b	.L398
.L397:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x0]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	lsl	w0, w0, 6
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	cmp	w1, w0
	bgt	.L399
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x0]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	lsl	w0, w0, 6
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	cmp	w1, w0
	bgt	.L399
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	ldrb	w0, [x0]
	and	w0, w0, 8
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L400
.L399:
	str	wzr, [sp, 156]
	b	.L401
.L406:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L402
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mul	w0, w1, w0
	lsl	w0, w0, 6
	adrp	x1, screen_x_max
	add	x1, x1, :lo12:screen_x_max
	ldr	w1, [x1]
	udiv	w0, w0, w1
	mov	w2, w0
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	w2, [x0]
.L402:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L403
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w1, [x0]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	sub	w0, w1, w0
	mov	w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mul	w0, w1, w0
	lsl	w0, w0, 6
	adrp	x1, screen_x_max
	add	x1, x1, :lo12:screen_x_max
	ldr	w1, [x1]
	udiv	w0, w0, w1
	mov	w2, w0
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	w2, [x0]
.L403:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L404
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mul	w0, w1, w0
	lsl	w0, w0, 6
	adrp	x1, screen_y_max
	add	x1, x1, :lo12:screen_y_max
	ldr	w1, [x1]
	udiv	w0, w0, w1
	mov	w2, w0
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	w2, [x0]
.L404:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L405
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w1, [x0]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	sub	w0, w1, w0
	mov	w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mul	w0, w1, w0
	lsl	w0, w0, 6
	adrp	x1, screen_y_max
	add	x1, x1, :lo12:screen_y_max
	ldr	w1, [x1]
	udiv	w0, w0, w1
	mov	w2, w0
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	w2, [x0]
.L405:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L401:
	ldr	w0, [sp, 156]
	cmp	w0, 3
	ble	.L406
	b	.L398
.L400:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L396:
	ldr	w0, [sp, 156]
	cmp	w0, 3
	ble	.L407
.L398:
	str	wzr, [sp, 152]
	b	.L408
.L440:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 152]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L409
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 152]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L410
.L409:
	b	.L411
.L410:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 152]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [sp, 148]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 152]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [sp, 144]
	str	wzr, [sp, 128]
	ldr	w0, [sp, 144]
	str	w0, [sp, 132]
	str	wzr, [sp, 156]
	b	.L412
.L416:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L413
	b	.L414
.L413:
	ldr	w1, [sp, 132]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L415
	ldr	w1, [sp, 128]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w2, [x0]
	ldr	w0, [sp, 132]
	sub	w2, w2, w0
	ldr	x3, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x3, x0
	ldr	w0, [x0, 4]
	mul	w0, w2, w0
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	add	w0, w1, w0
	str	w0, [sp, 128]
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 8
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	str	w0, [sp, 132]
.L415:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L412:
	ldr	w0, [sp, 156]
	cmp	w0, 3
	ble	.L416
.L414:
	ldr	w1, [sp, 132]
	ldr	w0, [sp, 128]
	sub	w0, w1, w0
	str	w0, [sp, 144]
	ldr	w0, [sp, 144]
	cmp	w0, wzr
	bgt	.L417
	mov	w0, 1
	str	w0, [sp, 144]
.L417:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 144]
	cmp	w1, w0
	bgt	.L418
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 144]
.L418:
	str	wzr, [sp, 128]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 144]
	sub	w0, w1, w0
	str	w0, [sp, 132]
	str	wzr, [sp, 156]
	b	.L419
.L423:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L420
	b	.L421
.L420:
	ldr	w1, [sp, 132]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L422
	ldr	w1, [sp, 128]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w2, [x0]
	ldr	w0, [sp, 132]
	sub	w2, w2, w0
	ldr	x3, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x3, x0
	ldr	w0, [x0, 4]
	mul	w0, w2, w0
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	add	w0, w1, w0
	str	w0, [sp, 128]
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 12
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	str	w0, [sp, 132]
.L422:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L419:
	ldr	w0, [sp, 156]
	cmp	w0, 3
	ble	.L423
.L421:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	ldr	w2, [sp, 128]
	ldr	w1, [sp, 132]
	sub	w1, w2, w1
	add	w0, w0, w1
	str	w0, [sp, 144]
	ldr	w0, [sp, 144]
	cmp	w0, wzr
	bgt	.L424
	mov	w0, 1
	str	w0, [sp, 144]
.L424:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 144]
	cmp	w1, w0
	bgt	.L425
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 144]
.L425:
	str	wzr, [sp, 128]
	ldr	w0, [sp, 148]
	str	w0, [sp, 132]
	str	wzr, [sp, 156]
	b	.L426
.L430:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L427
	b	.L428
.L427:
	ldr	w1, [sp, 132]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L429
	ldr	w1, [sp, 128]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w2, [x0]
	ldr	w0, [sp, 132]
	sub	w2, w2, w0
	ldr	x3, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x3, x0
	ldr	w0, [x0, 4]
	mul	w0, w2, w0
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	add	w0, w1, w0
	str	w0, [sp, 128]
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	str	w0, [sp, 132]
.L429:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L426:
	ldr	w0, [sp, 156]
	cmp	w0, 3
	ble	.L430
.L428:
	ldr	w1, [sp, 132]
	ldr	w0, [sp, 128]
	sub	w0, w1, w0
	str	w0, [sp, 148]
	ldr	w0, [sp, 148]
	cmp	w0, wzr
	bgt	.L431
	mov	w0, 1
	str	w0, [sp, 148]
.L431:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 148]
	cmp	w1, w0
	bgt	.L432
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 148]
.L432:
	str	wzr, [sp, 128]
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 148]
	sub	w0, w1, w0
	str	w0, [sp, 132]
	str	wzr, [sp, 156]
	b	.L433
.L437:
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L434
	b	.L435
.L434:
	ldr	w1, [sp, 132]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L436
	ldr	w1, [sp, 128]
	ldr	x2, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w2, [x0]
	ldr	w0, [sp, 132]
	sub	w2, w2, w0
	ldr	x3, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x3, x0
	ldr	w0, [x0, 4]
	mul	w0, w2, w0
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	add	w0, w1, w0
	str	w0, [sp, 128]
	ldr	x1, [sp, 136]
	ldrsw	x0, [sp, 156]
	add	x0, x0, 4
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0]
	str	w0, [sp, 132]
.L436:
	ldr	w0, [sp, 156]
	add	w0, w0, 1
	str	w0, [sp, 156]
.L433:
	ldr	w0, [sp, 156]
	cmp	w0, 3
	ble	.L437
.L435:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	ldr	w2, [sp, 128]
	ldr	w1, [sp, 132]
	sub	w1, w2, w1
	add	w0, w0, w1
	str	w0, [sp, 148]
	ldr	w0, [sp, 148]
	cmp	w0, wzr
	bgt	.L438
	mov	w0, 1
	str	w0, [sp, 148]
.L438:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 148]
	cmp	w1, w0
	bgt	.L439
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 148]
.L439:
	ldr	w0, [sp, 148]
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 152]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	ldr	w0, [sp, 144]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 152]
	lsl	x0, x0, 2
	add	x1, x1, x0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L411:
	ldr	w0, [sp, 152]
	add	w0, w0, 1
	str	w0, [sp, 152]
.L408:
	ldr	w0, [sp, 152]
	cmp	w0, 9
	ble	.L440
.L386:
	add	sp, sp, 160
	ret
	.size	PointEdge, .-PointEdge
	.align	2
	.type	PointStretch, %function
PointStretch:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	str	x0, [x29, 104]
	mov	w0, 8
	str	w0, [x29, 116]
	str	wzr, [x29, 124]
	b	.L443
.L444:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L443:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L444
	str	wzr, [x29, 124]
	b	.L445
.L466:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L446
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L447
.L446:
	ldr	w0, [x29, 124]
	mov	w1, 1
	lsl	w0, w1, w0
	mvn	w0, w0
	mov	w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	ldr	w0, [x0]
	and	w1, w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	str	w1, [x0]
	ldr	w0, [x29, 124]
	cmp	w0, wzr
	bne	.L448
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	str	wzr, [x0]
	b	.L449
.L448:
	b	.L449
.L447:
	ldr	w0, [x29, 124]
	cmp	w0, wzr
	bne	.L450
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L451
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L451
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [x0]
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	str	w1, [x0, 8]
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldrh	w0, [x0, 10]
	mov	w1, w0
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	lsr	w0, w0, 24
	cmp	w1, w0
	bcs	.L452
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	lsr	w0, w0, 24
	uxth	w1, w0
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	strh	w1, [x0, 10]
.L452:
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldrh	w0, [x0, 10]
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	lsr	w0, w0, 16
	and	w0, w0, 255
	sub	w0, w1, w0
	cmp	w2, w0
	bls	.L453
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	uxth	w0, w0
	lsl	w0, w0, 6
	uxth	w1, w0
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	lsr	w0, w0, 16
	uxth	w0, w0
	and	w0, w0, 255
	uxth	w0, w0
	sub	w0, w1, w0
	uxth	w1, w0
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	strh	w1, [x0, 10]
.L453:
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldrh	w0, [x0, 8]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	lsr	w0, w0, 8
	and	w0, w0, 255
	cmp	w1, w0
	bcs	.L454
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	lsr	w0, w0, 8
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w0, w0
	and	w0, w0, 255
	uxth	w2, w0
	adrp	x0, point_edge
	add	x1, x0, :lo12:point_edge
	ldrh	w0, [x1, 8]
	bfi	w0, w2, 0, 12
	strh	w0, [x1, 8]
.L454:
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldrh	w0, [x0, 8]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	and	w0, w0, 255
	sub	w0, w1, w0
	cmp	w2, w0
	bls	.L455
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	uxth	w0, w0
	lsl	w0, w0, 6
	uxth	w1, w0
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	ldr	w0, [x0]
	uxth	w0, w0
	and	w0, w0, 255
	uxth	w0, w0
	sub	w0, w1, w0
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	adrp	x0, point_edge
	add	x1, x0, :lo12:point_edge
	ldrh	w0, [x1, 8]
	bfi	w0, w2, 0, 12
	strh	w0, [x1, 8]
.L455:
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w1, [x0, 8]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	beq	.L451
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_edge+8
	add	x0, x0, :lo12:point_edge+8
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	mov	w1, w0
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	str	w1, [x0, 4]
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	beq	.L451
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	mov	w1, 4096
	str	w1, [x0]
.L451:
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L450
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	beq	.L450
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_edge+8
	add	x0, x0, :lo12:point_edge+8
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	str	w0, [x29, 100]
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w1, [x0, 4]
	adrp	x0, edge_first_coe
	add	x0, x0, :lo12:edge_first_coe
	ldr	w0, [x0]
	mul	w0, w1, w0
	lsr	w1, w0, 7
	ldr	w0, [x29, 100]
	cmp	w1, w0
	bhi	.L456
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	str	wzr, [x0]
	b	.L457
.L456:
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w1, [x0, 4]
	ldr	w0, [x29, 100]
	cmp	w1, w0
	bcs	.L457
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w1, [x0, 4]
	adrp	x0, edge_first_coe
	add	x0, x0, :lo12:edge_first_coe
	ldr	w0, [x0]
	mul	w0, w1, w0
	lsr	w1, w0, 7
	ldr	w0, [x29, 100]
	sub	w0, w1, w0
	lsl	w1, w0, 12
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w0, [x0, 4]
	udiv	w0, w1, w0
	str	w0, [x29, 100]
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w1, [x0]
	ldr	w0, [x29, 100]
	cmp	w1, w0
	bls	.L457
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w1, [x29, 100]
	str	w1, [x0]
.L457:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_edge
	add	x1, x1, :lo12:point_edge
	ldrh	w2, [x1, 10]
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x3, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x3, x1
	ldrh	w1, [x1, 2]
	mov	w3, w1
	adrp	x1, point_edge
	add	x1, x1, :lo12:point_edge
	ldrh	w1, [x1, 10]
	sub	w1, w3, w1
	mov	w4, w1
	adrp	x1, point_edge
	add	x1, x1, :lo12:point_edge
	ldr	w1, [x1]
	mov	w3, 4096
	sub	w1, w3, w1
	mul	w1, w4, w1
	lsr	w1, w1, 12
	uxth	w1, w1
	add	w1, w2, w1
	uxth	w1, w1
	strh	w1, [x0, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldrh	w0, [x0, 8]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldrh	w0, [x0, 8]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w0, w2, w0
	mov	w4, w0
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	ldr	w0, [x0]
	mov	w2, 4096
	sub	w0, w2, w0
	mul	w0, w4, w0
	lsr	w0, w0, 12
	uxth	w0, w0
	add	w0, w3, w0
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L450:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L458
	b	.L449
.L458:
	ldr	w0, [x29, 124]
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	ldr	w0, [x0]
	and	w0, w1, w0
	cmp	w0, wzr
	bne	.L449
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	mov	w1, w0
	adrp	x0, shake_min
	add	x0, x0, :lo12:shake_min
	ldr	w0, [x0]
	cmp	w1, w0
	bcs	.L459
	adrp	x0, point_delay
	add	x1, x0, :lo12:point_delay
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 6, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L460
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1, 8]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	b	.L461
.L460:
	mov	w0, 1
	str	w0, [x29, 120]
	b	.L462
.L463:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 120]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L462:
	ldr	w0, [x29, 120]
	cmp	w0, 9
	ble	.L463
	str	wzr, [x29, 120]
	b	.L464
.L465:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [x29, 120]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L464:
	ldr	w0, [x29, 120]
	cmp	w0, 9
	ble	.L465
.L461:
	b	.L449
.L459:
	ldr	w0, [x29, 124]
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	ldr	w0, [x0]
	orr	w1, w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	str	w1, [x0]
.L449:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L445:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L466
	str	wzr, [x29, 124]
	b	.L467
.L470:
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 3
	ldr	x1, [x29, 104]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L468
	ldr	w0, [x29, 124]
	str	w0, [x29, 116]
	b	.L469
.L468:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L467:
	ldr	w1, [x29, 124]
	ldr	w0, [x29, 116]
	cmp	w1, w0
	blt	.L470
.L469:
	ldr	w0, [x29, 116]
	cmp	w0, 1
	bne	.L471
	ldr	x0, [x29, 104]
	ldr	w1, [x0]
	add	x0, x29, 16
	str	w1, [x0]
	ldr	x0, [x29, 104]
	ldr	w1, [x0, 4]
	mov	w0, 100
	mul	w0, w1, w0
	add	w0, w0, 64
	add	w1, w0, 127
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 7
	mov	w1, w0
	add	x0, x29, 56
	str	w1, [x0]
	str	wzr, [x29, 124]
	b	.L472
.L488:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L473
	mov	w0, 1
	str	w0, [x29, 120]
	b	.L474
.L475:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 120]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L474:
	ldr	w0, [x29, 120]
	cmp	w0, 9
	ble	.L475
	b	.L476
.L473:
	ldr	w0, [x29, 124]
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	ldr	w0, [x0]
	and	w0, w1, w0
	cmp	w0, wzr
	bne	.L477
	b	.L476
.L477:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	str	w0, [x29, 96]
	ldr	w0, [x29, 96]
	bl	Sqrt
	str	w0, [x29, 96]
	add	x0, x29, 16
	ldr	w1, [x0]
	ldr	w0, [x29, 96]
	cmp	w1, w0
	bge	.L478
	add	x0, x29, 16
	ldr	w0, [x0]
	ldr	w1, [x29, 96]
	sub	w0, w1, w0
	b	.L479
.L478:
	mov	w0, 0
.L479:
	str	w0, [x29, 92]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	str	w0, [x29, 100]
	ldr	w0, [x29, 96]
	cmp	w0, wzr
	beq	.L480
	ldr	w0, [x29, 92]
	cmp	w0, wzr
	bne	.L481
.L480:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1, 8]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	strh	w1, [x0, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L482
.L481:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1, 8]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w2, [x1, 2]
	adrp	x1, point_pointer
	add	x1, x1, :lo12:point_pointer
	ldr	x3, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x3, x1
	ldrh	w1, [x1, 2]
	mov	w4, w1
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x3, [x1, 8]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x3, x1
	ldrh	w1, [x1, 2]
	sub	w3, w4, w1
	ldr	w1, [x29, 92]
	mul	w3, w3, w1
	ldr	w1, [x29, 96]
	sdiv	w1, w3, w1
	uxth	w1, w1
	add	w1, w2, w1
	uxth	w1, w1
	strh	w1, [x0, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w4, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w2, w4, w0
	ldr	w0, [x29, 92]
	mul	w2, w2, w0
	ldr	w0, [x29, 96]
	sdiv	w0, w2, w0
	uxth	w0, w0
	add	w0, w3, w0
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L482:
	add	x0, x29, 56
	ldr	w0, [x0]
	cmp	w0, wzr
	ble	.L476
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [x0]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bne	.L483
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [x0]
	ldr	w0, [x29, 100]
	cmp	w1, w0
	beq	.L483
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [x29, 100]
	str	w1, [x0]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrsh	w0, [x0, 2]
	uxth	w1, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w2, [x0, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x3, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x3, x0
	ldrh	w0, [x0, 2]
	sub	w0, w2, w0
	uxth	w0, w0
	add	w0, w1, w0
	uxth	w0, w0
	sxth	w2, w0
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0, 2]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrsh	w0, [x0]
	uxth	w1, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w0, w3, w0
	uxth	w0, w0
	add	w0, w1, w0
	uxth	w0, w0
	sxth	w2, w0
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	w1, w2
	strh	w1, [x0]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1, 8]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	strh	w1, [x0, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrsh	w0, [x0, 2]
	mov	w1, w0
	add	x0, x29, 56
	ldr	w0, [x0]
	cmp	w1, w0
	ble	.L484
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w1, [x0, 2]
	mov	w0, 65535
	cmp	w1, w0
	beq	.L484
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	add	w1, w1, 1
	uxth	w1, w1
	strh	w1, [x0, 2]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strh	wzr, [x0, 2]
.L484:
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrsh	w0, [x0, 2]
	mov	w1, w0
	add	x0, x29, 56
	ldr	w0, [x0]
	neg	w0, w0
	cmp	w1, w0
	bge	.L485
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	cmp	w0, wzr
	beq	.L485
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	sub	w1, w1, #1
	uxth	w1, w1
	strh	w1, [x0, 2]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strh	wzr, [x0, 2]
.L485:
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrsh	w0, [x0]
	mov	w1, w0
	add	x0, x29, 56
	ldr	w0, [x0]
	cmp	w1, w0
	ble	.L486
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	mov	w0, 65534
	cmp	w1, w0
	bgt	.L486
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	add	w0, w0, 1
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strh	wzr, [x0]
.L486:
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrsh	w0, [x0]
	mov	w1, w0
	add	x0, x29, 56
	ldr	w0, [x0]
	neg	w0, w0
	cmp	w1, w0
	bge	.L487
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	cmp	w0, wzr
	ble	.L487
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w0, w0, #1
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strh	wzr, [x0]
	b	.L476
.L487:
	b	.L476
.L483:
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strh	wzr, [x0, 2]
	adrp	x0, point_decimal
	add	x1, x0, :lo12:point_decimal
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	strh	wzr, [x0]
.L476:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L472:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L488
	b	.L442
.L471:
	ldr	w0, [x29, 116]
	cmp	w0, 1
	ble	.L442
	str	wzr, [x29, 124]
	b	.L491
.L493:
	ldr	w0, [x29, 124]
	add	w3, w0, 1
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 3
	ldr	x1, [x29, 104]
	add	x0, x1, x0
	ldr	w2, [x0]
	add	x0, x29, 16
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 124]
	add	w3, w0, 1
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 3
	ldr	x1, [x29, 104]
	add	x0, x1, x0
	ldr	w2, [x0, 4]
	add	x0, x29, 56
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L491:
	ldr	w0, [x29, 124]
	cmp	w0, 7
	bgt	.L492
	ldr	w1, [x29, 124]
	ldr	w0, [x29, 116]
	cmp	w1, w0
	blt	.L493
.L492:
	ldr	x0, [x29, 104]
	ldr	w0, [x0, 4]
	cmp	w0, 127
	bgt	.L494
	ldr	x0, [x29, 104]
	ldr	w1, [x0, 4]
	ldr	x0, [x29, 104]
	add	x0, x0, 8
	ldr	w0, [x0, 4]
	cmp	w1, w0
	bgt	.L495
.L494:
	add	x0, x29, 16
	ldr	w1, [x0, 4]
	add	x0, x29, 16
	str	w1, [x0]
	add	x0, x29, 56
	ldr	w1, [x0, 4]
	add	x0, x29, 56
	str	w1, [x0]
	b	.L496
.L495:
	add	x0, x29, 16
	ldr	w1, [x0, 4]
	ldr	x0, [x29, 104]
	ldr	w0, [x0, 4]
	mov	w2, 128
	sub	w2, w2, w0
	add	x0, x29, 16
	ldr	w3, [x0, 4]
	add	x0, x29, 16
	ldr	w0, [x0, 8]
	sub	w0, w3, w0
	mul	w2, w2, w0
	ldr	x0, [x29, 104]
	ldr	w3, [x0, 4]
	ldr	x0, [x29, 104]
	add	x0, x0, 8
	ldr	w0, [x0, 4]
	sub	w0, w3, w0
	sdiv	w0, w2, w0
	add	w1, w1, w0
	add	x0, x29, 16
	str	w1, [x0]
	add	x0, x29, 56
	mov	w1, 128
	str	w1, [x0]
.L496:
	str	wzr, [x29, 124]
	b	.L497
.L510:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L498
	mov	w0, 1
	str	w0, [x29, 120]
	b	.L499
.L500:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldrsw	x1, [x29, 120]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L499:
	ldr	w0, [x29, 120]
	cmp	w0, 9
	ble	.L500
	adrp	x0, save_dr.3292
	add	x0, x0, :lo12:save_dr.3292
	ldrsw	x1, [x29, 124]
	mov	w2, 128
	str	w2, [x0, x1, lsl 2]
	adrp	x0, save_dn.3293
	add	x0, x0, :lo12:save_dn.3293
	ldrsw	x1, [x29, 124]
	str	wzr, [x0, x1, lsl 2]
	b	.L501
.L498:
	ldr	w0, [x29, 124]
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	ldr	w0, [x0]
	and	w0, w1, w0
	cmp	w0, wzr
	bne	.L502
	b	.L501
.L502:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	str	w0, [x29, 96]
	ldr	w0, [x29, 96]
	bl	Sqrt
	str	w0, [x29, 96]
	add	x0, x29, 16
	ldr	w1, [x0]
	ldr	w0, [x29, 96]
	cmp	w1, w0
	bgt	.L503
	b	.L501
.L503:
	adrp	x0, save_dn.3293
	add	x0, x0, :lo12:save_dn.3293
	ldrsw	x1, [x29, 124]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 96]
	cmp	w1, w0
	ble	.L504
	adrp	x0, save_dr.3292
	add	x0, x0, :lo12:save_dr.3292
	ldrsw	x1, [x29, 124]
	ldr	w0, [x0, x1, lsl 2]
	str	w0, [x29, 92]
	adrp	x0, save_dn.3293
	add	x0, x0, :lo12:save_dn.3293
	ldrsw	x1, [x29, 124]
	ldr	w2, [x29, 96]
	str	w2, [x0, x1, lsl 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w2, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x3, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x3, x0
	ldrh	w0, [x0, 2]
	mov	w4, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x3, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x3, x0
	ldrh	w0, [x0, 2]
	sub	w3, w4, w0
	ldr	w0, [x29, 92]
	mul	w0, w3, w0
	add	w3, w0, 127
	cmp	w0, wzr
	csel	w0, w3, w0, lt
	asr	w0, w0, 7
	uxth	w0, w0
	add	w0, w2, w0
	uxth	w0, w0
	strh	w0, [x1, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w4, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w2, w4, w0
	ldr	w0, [x29, 92]
	mul	w0, w2, w0
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	uxth	w0, w0
	add	w0, w3, w0
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L501
.L504:
	str	wzr, [x29, 120]
	b	.L505
.L509:
	ldr	w1, [x29, 120]
	ldr	w0, [x29, 116]
	cmp	w1, w0
	beq	.L506
	ldr	w0, [x29, 96]
	cmp	w0, wzr
	bne	.L507
.L506:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, point_stretch
	add	x1, x1, :lo12:point_stretch
	ldr	x2, [x1, 8]
	ldrsw	x1, [x29, 124]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	strh	w1, [x0, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L501
.L507:
	add	x0, x29, 16
	ldrsw	x1, [x29, 120]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 96]
	cmp	w1, w0
	ble	.L508
	ldr	w0, [x29, 120]
	add	w1, w0, 1
	add	x0, x29, 16
	sxtw	x1, w1
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 96]
	cmp	w1, w0
	bgt	.L508
	ldr	w0, [x29, 120]
	add	w1, w0, 1
	add	x0, x29, 56
	sxtw	x1, w1
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 120]
	add	w2, w0, 1
	add	x0, x29, 16
	sxtw	x2, w2
	ldr	w0, [x0, x2, lsl 2]
	ldr	w2, [x29, 96]
	sub	w2, w2, w0
	add	x0, x29, 56
	ldrsw	x3, [x29, 120]
	ldr	w3, [x0, x3, lsl 2]
	ldr	w0, [x29, 120]
	add	w4, w0, 1
	add	x0, x29, 56
	sxtw	x4, w4
	ldr	w0, [x0, x4, lsl 2]
	sub	w0, w3, w0
	mul	w2, w2, w0
	add	x0, x29, 16
	ldrsw	x3, [x29, 120]
	ldr	w3, [x0, x3, lsl 2]
	ldr	w0, [x29, 120]
	add	w4, w0, 1
	add	x0, x29, 16
	sxtw	x4, w4
	ldr	w0, [x0, x4, lsl 2]
	sub	w0, w3, w0
	sdiv	w0, w2, w0
	add	w0, w1, w0
	str	w0, [x29, 92]
	adrp	x0, save_dr.3292
	add	x0, x0, :lo12:save_dr.3292
	ldrsw	x1, [x29, 124]
	ldr	w2, [x29, 92]
	str	w2, [x0, x1, lsl 2]
	adrp	x0, save_dn.3293
	add	x0, x0, :lo12:save_dn.3293
	ldrsw	x1, [x29, 124]
	ldr	w2, [x29, 96]
	str	w2, [x0, x1, lsl 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w2, [x0, 2]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x3, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x3, x0
	ldrh	w0, [x0, 2]
	mov	w4, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x3, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x3, x0
	ldrh	w0, [x0, 2]
	sub	w3, w4, w0
	ldr	w0, [x29, 92]
	mul	w0, w3, w0
	add	w0, w0, 64
	add	w3, w0, 127
	cmp	w0, wzr
	csel	w0, w3, w0, lt
	asr	w0, w0, 7
	uxth	w0, w0
	add	w0, w2, w0
	uxth	w0, w0
	strh	w0, [x1, 2]
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x1, x1, x0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x2, [x0]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w4, w0
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x2, [x0, 8]
	ldrsw	x0, [x29, 124]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w2, w4, w0
	ldr	w0, [x29, 92]
	mul	w0, w2, w0
	add	w0, w0, 64
	add	w2, w0, 127
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 7
	uxth	w0, w0
	add	w0, w3, w0
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	b	.L501
.L508:
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	str	w0, [x29, 120]
.L505:
	ldr	w1, [x29, 120]
	ldr	w0, [x29, 116]
	cmp	w1, w0
	ble	.L509
.L501:
	ldr	w0, [x29, 124]
	add	w0, w0, 1
	str	w0, [x29, 124]
.L497:
	ldr	w0, [x29, 124]
	cmp	w0, 9
	ble	.L510
.L442:
	ldp	x29, x30, [sp], 128
	ret
	.size	PointStretch, .-PointStretch
	.align	2
	.type	ResetMask, %function
ResetMask:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	adrp	x0, reset_mask_send
	add	x0, x0, :lo12:reset_mask_send
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L513
	adrp	x0, reset_mask_send
	add	x0, x0, :lo12:reset_mask_send
	str	wzr, [x0]
.L513:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 64
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L514
	b	.L512
.L514:
	adrp	x0, reset_mask_dis
	add	x0, x0, :lo12:reset_mask_dis
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L516
	adrp	x0, reset_mask_type
	add	x0, x0, :lo12:reset_mask_type
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L517
.L516:
	b	.L512
.L517:
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	ldr	w0, [x0]
	cmn	w0, #15
	bne	.L518
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L519
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	mov	w1, -268435455
	str	w1, [x0]
	b	.L512
.L519:
	b	.L512
.L518:
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	ldr	w1, [x0]
	mov	w0, -268435456
	cmp	w1, w0
	bls	.L520
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	ldr	w0, [x0]
	sub	w1, w0, #1
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	str	w1, [x0]
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	ldr	w1, [x0]
	mov	w0, -268435456
	cmp	w1, w0
	bne	.L521
	adrp	x0, reset_mask_type
	add	x0, x0, :lo12:reset_mask_type
	ldr	w1, [x0]
	adrp	x0, reset_mask_send
	add	x0, x0, :lo12:reset_mask_send
	str	w1, [x0]
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	orr	w1, w1, 64
	strb	w1, [x0, 3]
	b	.L512
.L521:
	b	.L512
.L520:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, 1
	bgt	.L522
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L523
.L522:
	adrp	x0, reset_mask_count
	add	x0, x0, :lo12:reset_mask_count
	str	wzr, [x0]
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	str	wzr, [x0]
	adrp	x0, reset_mask_count
	add	x0, x0, :lo12:reset_mask_count
	str	wzr, [x0]
	b	.L512
.L523:
	adrp	x0, reset_mask_count
	add	x0, x0, :lo12:reset_mask_count
	ldr	w0, [x0]
	add	w1, w0, 1
	adrp	x0, reset_mask_count
	add	x0, x0, :lo12:reset_mask_count
	str	w1, [x0]
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L524
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w1, [x0]
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	str	w1, [x0]
	b	.L512
.L524:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	bl	PointDistance
	mov	w1, w0
	adrp	x0, reset_mask_dis
	add	x0, x0, :lo12:reset_mask_dis
	ldr	w0, [x0]
	and	w0, w0, 16777215
	cmp	w1, w0
	bls	.L512
	adrp	x0, reset_mask_dis
	add	x0, x0, :lo12:reset_mask_dis
	ldr	w0, [x0]
	lsr	w1, w0, 24
	adrp	x0, reset_mask_count
	add	x0, x0, :lo12:reset_mask_count
	ldr	w0, [x0]
	cmp	w1, w0
	bcs	.L512
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	mov	w1, -15
	str	w1, [x0]
.L512:
	ldp	x29, x30, [sp], 16
	ret
	.size	ResetMask, .-ResetMask
	.align	2
	.type	ConfigCoorMulti, %function
ConfigCoorMulti:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	wzr, [sp, 20]
	str	wzr, [sp, 28]
	b	.L526
.L532:
	ldr	w0, [sp, 28]
	add	w0, w0, 247
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L527
	ldr	w0, [sp, 28]
	add	w0, w0, 247
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w0, w0, 63
	cmp	w0, wzr
	bne	.L528
	ldr	w0, [sp, 28]
	add	w0, w0, 247
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	asr	w0, w0, 16
	cmp	w0, 3
	bgt	.L528
	ldr	w0, [sp, 20]
	add	w0, w0, 1
	str	w0, [sp, 20]
	b	.L527
.L528:
	mov	w0, 0
	b	.L529
.L527:
	ldr	w0, [sp, 28]
	add	w0, w0, 251
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L530
	ldr	w0, [sp, 28]
	add	w0, w0, 251
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w0, w0, 63
	cmp	w0, wzr
	bne	.L531
	ldr	w0, [sp, 28]
	add	w0, w0, 251
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	asr	w0, w0, 16
	cmp	w0, 3
	bgt	.L531
	ldr	w0, [sp, 20]
	add	w0, w0, 1
	str	w0, [sp, 20]
	b	.L530
.L531:
	mov	w0, 0
	b	.L529
.L530:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L526:
	ldr	w0, [sp, 28]
	cmp	w0, 3
	ble	.L532
	ldr	w0, [sp, 20]
	cmp	w0, wzr
	beq	.L533
	ldr	w0, [sp, 20]
	cmp	w0, 4
	ble	.L534
.L533:
	mov	w0, 0
	b	.L529
.L534:
	str	wzr, [sp, 24]
	b	.L535
.L540:
	str	wzr, [sp, 28]
	b	.L536
.L539:
	ldr	w0, [sp, 24]
	add	w0, w0, 4
	lsl	w1, w0, 6
	ldr	w0, [sp, 28]
	add	w0, w1, w0
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, 63
	ble	.L537
	mov	w0, 0
	b	.L529
.L537:
	ldr	w0, [sp, 28]
	cmp	w0, wzr
	beq	.L538
	ldr	w0, [sp, 24]
	add	w0, w0, 4
	lsl	w1, w0, 6
	ldr	w0, [sp, 28]
	add	w0, w1, w0
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w1, [x0]
	ldr	w0, [sp, 24]
	add	w0, w0, 4
	lsl	w2, w0, 6
	ldr	w0, [sp, 28]
	add	w0, w2, w0
	sxtw	x0, w0
	lsl	x0, x0, 2
	sub	x0, x0, #4
	ldr	x2, [sp, 8]
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L538
	mov	w0, 0
	b	.L529
.L538:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L536:
	ldr	w0, [sp, 28]
	cmp	w0, 63
	ble	.L539
	ldr	w0, [sp, 24]
	add	w0, w0, 1
	str	w0, [sp, 24]
.L535:
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 20]
	cmp	w1, w0
	blt	.L540
	mov	w0, 1
.L529:
	add	sp, sp, 32
	ret
	.size	ConfigCoorMulti, .-ConfigCoorMulti
	.align	2
	.type	ConfigFilter, %function
ConfigFilter:
	sub	sp, sp, #96
	str	x0, [sp, 8]
	str	wzr, [sp, 88]
	ldr	x0, [sp, 8]
	add	x0, x0, 968
	ldr	w0, [x0]
	cmp	w0, 1
	bls	.L542
	ldr	x0, [sp, 8]
	add	x0, x0, 1020
	ldr	w0, [x0]
	cmp	w0, 256
	bhi	.L542
	str	wzr, [sp, 92]
	b	.L543
.L546:
	ldr	w0, [sp, 92]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	add	w0, w0, 243
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w2, [x0]
	ldr	w1, [sp, 92]
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	add	x0, sp, 56
	ldrsw	x1, [sp, 92]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [sp, 92]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	add	w0, w0, 243
	sxtw	x0, w0
	add	x0, x0, 2
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w2, [x0]
	ldr	w1, [sp, 92]
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	add	x0, sp, 24
	ldrsw	x1, [sp, 92]
	str	w2, [x0, x1, lsl 2]
	add	x0, sp, 56
	ldrsw	x1, [sp, 92]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 127
	bls	.L544
	add	x0, sp, 56
	ldrsw	x1, [sp, 92]
	ldr	w0, [x0, x1, lsl 2]
	orr	w2, w0, -256
	add	x0, sp, 56
	ldrsw	x1, [sp, 92]
	str	w2, [x0, x1, lsl 2]
.L544:
	add	x0, sp, 24
	ldrsw	x1, [sp, 92]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 127
	bls	.L545
	add	x0, sp, 24
	ldrsw	x1, [sp, 92]
	ldr	w0, [x0, x1, lsl 2]
	orr	w2, w0, -256
	add	x0, sp, 24
	ldrsw	x1, [sp, 92]
	str	w2, [x0, x1, lsl 2]
.L545:
	add	x0, sp, 56
	ldrsw	x1, [sp, 92]
	ldr	w0, [x0, x1, lsl 2]
	ldr	w1, [sp, 88]
	add	w0, w1, w0
	str	w0, [sp, 88]
	add	x0, sp, 24
	ldrsw	x1, [sp, 92]
	ldr	w0, [x0, x1, lsl 2]
	ldr	w1, [sp, 88]
	add	w0, w1, w0
	str	w0, [sp, 88]
	ldr	w0, [sp, 92]
	add	w0, w0, 1
	str	w0, [sp, 92]
.L543:
	ldr	w0, [sp, 92]
	cmp	w0, 7
	ble	.L546
	ldr	x0, [sp, 8]
	add	x0, x0, 968
	ldr	w1, [x0]
	ldr	w0, [sp, 88]
	cmp	w1, w0
	beq	.L547
	ldr	x0, [sp, 8]
	add	x0, x0, 968
	ldr	w1, [x0]
	ldr	w0, [sp, 88]
	add	w0, w1, w0
	cmp	w0, wzr
	bne	.L542
.L547:
	mov	w0, 1
	b	.L549
.L542:
	mov	w0, 0
.L549:
	add	sp, sp, 96
	ret
	.size	ConfigFilter, .-ConfigFilter
	.align	2
	.type	ConfigKeyMap, %function
ConfigKeyMap:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	ldr	x0, [sp, 8]
	add	x0, x0, 868
	ldr	w0, [x0]
	cmp	w0, 1
	beq	.L551
	mov	w0, 0
	b	.L552
.L551:
	str	wzr, [sp, 28]
	b	.L553
.L557:
	ldr	x0, [sp, 8]
	add	x0, x0, 880
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L554
	mov	w0, 0
	b	.L552
.L554:
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	add	x0, x0, 872
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	asr	w2, w0, 16
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	add	x0, x0, 872
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w0, w0, 65535
	cmp	w2, w0
	ble	.L555
	mov	w0, 0
	b	.L552
.L555:
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	add	x0, x0, 876
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	asr	w2, w0, 16
	ldrsw	x1, [sp, 28]
	mov	x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x1
	lsl	x0, x0, 2
	add	x0, x0, 876
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w0, w0, 65535
	cmp	w2, w0
	ble	.L556
	mov	w0, 0
	b	.L552
.L556:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L553:
	ldr	w0, [sp, 28]
	cmp	w0, 7
	ble	.L557
	mov	w0, 1
.L552:
	add	sp, sp, 32
	ret
	.size	ConfigKeyMap, .-ConfigKeyMap
	.align	2
	.type	DiagonalDistance, %function
DiagonalDistance:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	w1, [sp, 4]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mul	w0, w2, w0
	add	w0, w1, w0
	add	w1, w0, 15
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 4
	str	w0, [sp, 28]
	ldr	w0, [sp, 28]
	cmp	w0, wzr
	bne	.L559
	mov	w0, 1
	str	w0, [sp, 28]
.L559:
	ldr	w0, [sp, 4]
	cmp	w0, wzr
	bne	.L560
	ldr	x0, [sp, 8]
	ldrh	w0, [x0, 2]
	mov	w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mul	w1, w1, w0
	ldr	x0, [sp, 8]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mul	w0, w2, w0
	sub	w0, w1, w0
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	str	w0, [sp, 24]
	b	.L561
.L560:
	ldr	x0, [sp, 8]
	ldrh	w0, [x0, 2]
	mov	w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mul	w1, w1, w0
	ldr	x0, [sp, 8]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mul	w0, w2, w0
	add	w2, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mul	w1, w1, w0
	mov	w0, w1
	lsl	w1, w1, 1
	sub	w0, w0, w1
	lsl	w0, w0, 6
	add	w0, w2, w0
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	str	w0, [sp, 24]
.L561:
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 24]
	mul	w1, w1, w0
	ldr	w0, [sp, 28]
	sdiv	w0, w1, w0
	add	sp, sp, 32
	ret
	.size	DiagonalDistance, .-DiagonalDistance
	.align	2
	.type	DiagonalCompress, %function
DiagonalCompress:
	sub	sp, sp, #64
	str	x0, [sp, 24]
	str	w1, [sp, 20]
	str	w2, [sp, 16]
	str	w3, [sp, 12]
	ldr	w0, [sp, 12]
	cmp	w0, wzr
	bne	.L564
	b	.L563
.L564:
	ldr	w1, [sp, 16]
	ldr	w0, [sp, 12]
	cmp	w1, w0
	ble	.L566
	ldr	w1, [sp, 16]
	ldr	w0, [sp, 12]
	sub	w0, w1, w0
	lsl	w2, w0, 7
	ldr	w1, [sp, 12]
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	sdiv	w0, w2, w0
	add	w0, w0, 128
	str	w0, [sp, 52]
	b	.L567
.L566:
	mov	w0, 128
	str	w0, [sp, 52]
.L567:
	ldr	w0, [sp, 52]
	cmp	w0, 256
	ble	.L568
	mov	w0, 256
	str	w0, [sp, 52]
.L568:
	ldr	x0, [sp, 24]
	ldrh	w0, [x0, 2]
	str	w0, [sp, 60]
	ldr	x0, [sp, 24]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [sp, 56]
	ldr	w0, [sp, 20]
	cmp	w0, wzr
	beq	.L569
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 56]
	sub	w0, w1, w0
	str	w0, [sp, 56]
.L569:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [sp, 60]
	mul	w0, w0, w1
	str	w0, [sp, 60]
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [sp, 56]
	mul	w0, w0, w1
	str	w0, [sp, 56]
	ldr	w0, [sp, 60]
	str	w0, [sp, 48]
	ldr	w0, [sp, 56]
	str	w0, [sp, 44]
	ldr	w1, [sp, 48]
	ldr	w0, [sp, 44]
	add	w1, w1, w0
	ldr	w2, [sp, 48]
	ldr	w0, [sp, 44]
	sub	w2, w2, w0
	ldr	w0, [sp, 52]
	mul	w0, w2, w0
	add	w2, w0, 255
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 8
	add	w0, w1, w0
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [sp, 60]
	ldr	w1, [sp, 48]
	ldr	w0, [sp, 44]
	add	w1, w1, w0
	ldr	w2, [sp, 44]
	ldr	w0, [sp, 48]
	sub	w2, w2, w0
	ldr	w0, [sp, 52]
	mul	w0, w2, w0
	add	w2, w0, 255
	cmp	w0, wzr
	csel	w0, w2, w0, lt
	asr	w0, w0, 8
	add	w0, w1, w0
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [sp, 56]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [sp, 60]
	sdiv	w0, w0, w1
	str	w0, [sp, 60]
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [sp, 56]
	sdiv	w0, w0, w1
	str	w0, [sp, 56]
	ldr	w0, [sp, 20]
	cmp	w0, wzr
	beq	.L570
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 56]
	sub	w0, w1, w0
	str	w0, [sp, 56]
.L570:
	ldr	w0, [sp, 60]
	cmp	w0, wzr
	bgt	.L571
	mov	w0, 1
	str	w0, [sp, 60]
.L571:
	ldr	w0, [sp, 56]
	cmp	w0, wzr
	bgt	.L572
	mov	w0, 1
	str	w0, [sp, 56]
.L572:
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 60]
	cmp	w1, w0
	bgt	.L573
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 60]
.L573:
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [sp, 56]
	cmp	w1, w0
	bgt	.L574
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sub	w0, w0, #1
	str	w0, [sp, 56]
.L574:
	ldr	w0, [sp, 60]
	uxth	w1, w0
	ldr	x0, [sp, 24]
	strh	w1, [x0, 2]
	ldr	w0, [sp, 56]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldr	x1, [sp, 24]
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
.L563:
	add	sp, sp, 64
	ret
	.size	DiagonalCompress, .-DiagonalCompress
	.align	2
	.type	PointDiagonal, %function
PointDiagonal:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L576
	b	.L575
.L576:
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	ldr	w1, [x0]
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	ldr	w0, [x0]
	mul	w0, w1, w0
	str	w0, [x29, 24]
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	ldr	w1, [x0]
	mov	w0, w1
	lsl	w0, w0, 1
	add	w0, w0, w1
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [x29, 20]
	str	wzr, [x29, 28]
	b	.L578
.L590:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L579
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L580
.L579:
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 3
	lsl	w0, w1, w0
	mvn	w0, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	and	w1, w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	w1, [x0]
	b	.L581
.L580:
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 3
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	and	w0, w1, w0
	cmp	w0, wzr
	bne	.L582
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 20]
	cmp	w1, w0
	bhi	.L583
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 20]
	cmp	w1, w0
	bls	.L584
.L583:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 20]
	sub	w0, w1, w0
	cmp	w2, w0
	bcc	.L585
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 20]
	sub	w0, w1, w0
	cmp	w2, w0
	bcc	.L585
.L584:
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 2
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	orr	w1, w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	w1, [x0]
	b	.L582
.L585:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 20]
	cmp	w1, w0
	bhi	.L586
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 20]
	sub	w0, w1, w0
	cmp	w2, w0
	bcs	.L587
.L586:
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 20]
	sub	w0, w1, w0
	cmp	w2, w0
	bcc	.L588
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 20]
	cmp	w1, w0
	bhi	.L588
.L587:
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 3
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	orr	w1, w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	w1, [x0]
	b	.L582
.L588:
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	orr	w1, w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	w1, [x0]
.L582:
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 2
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	and	w0, w1, w0
	cmp	w0, wzr
	beq	.L581
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x2, x1, x0
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	and	w0, w1, w0
	mov	w1, w0
	mov	x0, x2
	bl	DiagonalDistance
	str	w0, [x29, 16]
	ldr	w0, [x29, 24]
	lsl	w1, w0, 2
	ldr	w0, [x29, 16]
	cmp	w1, w0
	blt	.L589
	adrp	x0, point_stretch
	add	x0, x0, :lo12:point_stretch
	ldr	x1, [x0]
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	add	x4, x1, x0
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	and	w0, w1, w0
	ldr	w3, [x29, 24]
	ldr	w2, [x29, 16]
	mov	w1, w0
	mov	x0, x4
	bl	DiagonalCompress
	b	.L581
.L589:
	ldr	w0, [x29, 24]
	lsl	w1, w0, 2
	ldr	w0, [x29, 16]
	cmp	w1, w0
	bge	.L581
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 3
	lsl	w0, w1, w0
	mvn	w0, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	and	w1, w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	w1, [x0]
	ldr	w0, [x29, 28]
	lsl	w0, w0, 1
	mov	w1, 1
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	ldr	w0, [x0]
	orr	w1, w1, w0
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	w1, [x0]
.L581:
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L578:
	ldr	w0, [x29, 28]
	cmp	w0, 9
	ble	.L590
.L575:
	ldp	x29, x30, [sp], 32
	ret
	.size	PointDiagonal, .-PointDiagonal
	.align	2
	.type	PointSlope, %function
PointSlope:
	sub	sp, sp, #32
	str	w0, [sp, 12]
	str	w1, [sp, 8]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [sp, 8]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldr	w0, [sp, 8]
	add	w1, w0, 1
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	sub	w0, w2, w0
	str	w0, [sp, 28]
	ldr	w1, [sp, 28]
	ldr	w0, [sp, 28]
	mul	w0, w1, w0
	str	w0, [sp, 28]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [sp, 8]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldr	w0, [sp, 8]
	add	w1, w0, 1
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	sxtw	x1, w1
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w0, w2, w0
	str	w0, [sp, 24]
	ldr	w1, [sp, 24]
	ldr	w0, [sp, 24]
	mul	w0, w1, w0
	str	w0, [sp, 24]
	ldr	w1, [sp, 28]
	ldr	w0, [sp, 24]
	add	w0, w1, w0
	cmp	w0, wzr
	bne	.L592
	mov	w0, -1
	b	.L593
.L592:
	ldr	w1, [sp, 28]
	ldr	w0, [sp, 24]
	cmp	w1, w0
	ble	.L594
	ldr	w0, [sp, 28]
	lsl	w1, w0, 10
	ldr	w2, [sp, 28]
	ldr	w0, [sp, 24]
	add	w0, w2, w0
	sdiv	w0, w1, w0
	b	.L593
.L594:
	ldr	w0, [sp, 24]
	lsl	w1, w0, 10
	ldr	w2, [sp, 28]
	ldr	w0, [sp, 24]
	add	w0, w2, w0
	sdiv	w0, w1, w0
.L593:
	add	sp, sp, 32
	ret
	.size	PointSlope, .-PointSlope
	.align	2
	.type	PointExtend, %function
PointExtend:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	mov	w0, 5
	str	w0, [x29, 52]
	adrp	x0, point_extend
	add	x0, x0, :lo12:point_extend
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L596
	b	.L595
.L596:
	str	wzr, [x29, 60]
	b	.L598
.L611:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L599
	b	.L600
.L599:
	str	wzr, [x29, 56]
	b	.L601
.L604:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldrsw	x1, [x29, 56]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L602
	b	.L603
.L602:
	ldr	w0, [x29, 56]
	add	w0, w0, 1
	str	w0, [x29, 56]
.L601:
	ldr	w1, [x29, 56]
	ldr	w0, [x29, 52]
	cmp	w1, w0
	blt	.L604
.L603:
	ldr	w1, [x29, 56]
	ldr	w0, [x29, 52]
	cmp	w1, w0
	bge	.L605
	b	.L600
.L605:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x2, x1, x0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 16]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	mov	x1, x0
	mov	x0, x2
	bl	PointDistance
	cmp	w0, 255
	bhi	.L606
	b	.L600
.L606:
	mov	w1, 1
	ldr	w0, [x29, 60]
	bl	PointSlope
	str	w0, [x29, 48]
	mov	w0, 2
	str	w0, [x29, 56]
	b	.L607
.L609:
	ldr	w1, [x29, 56]
	ldr	w0, [x29, 60]
	bl	PointSlope
	str	w0, [x29, 44]
	ldr	w0, [x29, 44]
	cmp	w0, wzr
	blt	.L608
	adrp	x0, point_extend
	add	x0, x0, :lo12:point_extend
	ldr	w0, [x0]
	mov	w1, 128
	sub	w1, w1, w0
	ldr	w0, [x29, 48]
	mul	w0, w1, w0
	add	w1, w0, 127
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 7
	mov	w1, w0
	ldr	w0, [x29, 44]
	cmp	w1, w0
	bgt	.L608
	adrp	x0, point_extend
	add	x0, x0, :lo12:point_extend
	ldr	w0, [x0]
	add	w1, w0, 128
	ldr	w0, [x29, 48]
	mul	w0, w1, w0
	add	w1, w0, 127
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 7
	mov	w1, w0
	ldr	w0, [x29, 44]
	cmp	w1, w0
	blt	.L608
	ldr	w0, [x29, 56]
	add	w0, w0, 1
	str	w0, [x29, 56]
.L607:
	ldr	w0, [x29, 52]
	sub	w1, w0, #1
	ldr	w0, [x29, 56]
	cmp	w1, w0
	bgt	.L609
.L608:
	ldr	w0, [x29, 52]
	sub	w1, w0, #1
	ldr	w0, [x29, 56]
	cmp	w1, w0
	ble	.L610
	b	.L600
.L610:
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	mov	w0, w1
	lsl	w0, w0, 1
	add	w1, w0, w1
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x2, [x0, 16]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	mov	w0, w2
	lsl	w2, w2, 1
	sub	w0, w0, w2
	lsl	w0, w0, 1
	add	w0, w1, w0
	str	w0, [x29, 40]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	mov	w0, w1
	lsl	w0, w0, 1
	add	w1, w0, w1
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x2, [x0, 16]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	mov	w0, w2
	lsl	w2, w2, 1
	sub	w0, w0, w2
	lsl	w0, w0, 1
	add	w0, w1, w0
	str	w0, [x29, 36]
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x19, x1, x0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x1, [x0, 8]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldr	w1, [x29, 36]
	ldr	w0, [x29, 40]
	bl	PointRange
	str	w0, [x19]
.L600:
	ldr	w0, [x29, 60]
	add	w0, w0, 1
	str	w0, [x29, 60]
.L598:
	ldr	w0, [x29, 60]
	cmp	w0, 9
	ble	.L611
.L595:
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 64
	ret
	.size	PointExtend, .-PointExtend
	.align	2
	.type	PressureSave, %function
PressureSave:
	sub	sp, sp, #16
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 4096
	cmp	w0, wzr
	bne	.L613
	b	.L612
.L613:
	str	wzr, [sp, 12]
	b	.L615
.L616:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	ldr	w0, [x0, x1, lsl 2]
	lsr	w2, w0, 28
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [sp, 12]
	str	w2, [x0, x1, lsl 2]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	ldr	w0, [x0, x1, lsl 2]
	and	w2, w0, 268435455
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L615:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L616
.L612:
	add	sp, sp, 16
	ret
	.size	PressureSave, .-PressureSave
	.align	2
	.type	PointPressure, %function
PointPressure:
	sub	sp, sp, #16
	str	wzr, [sp, 12]
	b	.L618
.L626:
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L619
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x1, [x0, 8]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L619
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x1, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w1, [x0]
	mov	w0, w1
	lsl	w0, w0, 2
	add	w2, w0, w1
	adrp	x0, pressure_report
	add	x0, x0, :lo12:pressure_report
	ldrsw	x1, [sp, 12]
	str	w2, [x0, x1, lsl 2]
	mov	w0, 1
	str	w0, [sp, 8]
	b	.L620
.L621:
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldrsw	x1, [sp, 8]
	ldr	x1, [x0, x1, lsl 3]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	adrp	x1, pressure_pointer
	add	x1, x1, :lo12:pressure_pointer
	ldr	x2, [x1]
	ldrsw	x1, [sp, 12]
	lsl	x1, x1, 2
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [sp, 8]
	add	w0, w0, 1
	str	w0, [sp, 8]
.L620:
	ldr	w0, [sp, 8]
	cmp	w0, 7
	ble	.L621
	b	.L622
.L619:
	adrp	x0, pressure_report
	add	x0, x0, :lo12:pressure_report
	ldrsw	x1, [sp, 12]
	ldr	w0, [x0, x1, lsl 2]
	add	w0, w0, 1
	lsr	w1, w0, 1
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x2, [x0]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w0, [x0]
	add	w1, w1, w0
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x2, [x0, 8]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w0, [x0]
	add	w1, w1, w0
	adrp	x0, pressure_pointer
	add	x0, x0, :lo12:pressure_pointer
	ldr	x2, [x0, 16]
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w0, [x0]
	add	w0, w0, 1
	lsr	w0, w0, 1
	add	w1, w1, w0
	adrp	x0, pressure_report
	add	x0, x0, :lo12:pressure_report
	ldrsw	x2, [sp, 12]
	ldr	w0, [x0, x2, lsl 2]
	sub	w0, w1, w0
	str	w0, [sp, 8]
	ldr	w0, [sp, 8]
	cmp	w0, 1
	ble	.L623
	ldr	w0, [sp, 8]
	sub	w0, w0, #2
	str	w0, [sp, 8]
	b	.L624
.L623:
	ldr	w0, [sp, 8]
	cmn	w0, #1
	bge	.L625
	ldr	w0, [sp, 8]
	add	w0, w0, 2
	str	w0, [sp, 8]
	b	.L624
.L625:
	str	wzr, [sp, 8]
.L624:
	adrp	x0, pressure_report
	add	x0, x0, :lo12:pressure_report
	ldrsw	x1, [sp, 12]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [sp, 8]
	add	w2, w1, w0
	adrp	x0, pressure_report
	add	x0, x0, :lo12:pressure_report
	ldrsw	x1, [sp, 12]
	str	w2, [x0, x1, lsl 2]
.L622:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L618:
	ldr	w0, [sp, 12]
	cmp	w0, 9
	ble	.L626
	add	sp, sp, 16
	ret
	.size	PointPressure, .-PointPressure
	.align	2
	.global	gsl_ReportPressure
	.type	gsl_ReportPressure, %function
gsl_ReportPressure:
	sub	sp, sp, #32
	str	x0, [sp, 8]
	str	wzr, [sp, 28]
	b	.L628
.L635:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [sp, 28]
	cmp	w1, w0
	bge	.L629
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [sp, 28]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	bne	.L630
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	str	wzr, [x0]
	b	.L634
.L630:
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [sp, 28]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 7
	bhi	.L632
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	mov	w1, 1
	str	w1, [x0]
	b	.L634
.L632:
	adrp	x0, pressure_now
	add	x0, x0, :lo12:pressure_now
	ldrsw	x1, [sp, 28]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 70
	bls	.L633
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	mov	w1, 63
	str	w1, [x0]
	b	.L634
.L633:
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	adrp	x1, pressure_now
	add	x1, x1, :lo12:pressure_now
	ldrsw	x2, [sp, 28]
	ldr	w1, [x1, x2, lsl 2]
	sub	w1, w1, #7
	str	w1, [x0]
	b	.L634
.L629:
	ldrsw	x0, [sp, 28]
	lsl	x0, x0, 2
	ldr	x1, [sp, 8]
	add	x0, x1, x0
	str	wzr, [x0]
.L634:
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L628:
	ldr	w0, [sp, 28]
	cmp	w0, 9
	ble	.L635
	add	sp, sp, 32
	ret
	.size	gsl_ReportPressure, .-gsl_ReportPressure
	.align	2
	.global	gsl_TouchNear
	.type	gsl_TouchNear, %function
gsl_TouchNear:
	mov	w0, 0
	ret
	.size	gsl_TouchNear, .-gsl_TouchNear
	.align	2
	.type	gsl_id_reg_init, %function
gsl_id_reg_init:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	w0, [x29, 28]
	str	wzr, [x29, 40]
	b	.L639
.L642:
	str	wzr, [x29, 44]
	b	.L640
.L641:
	adrp	x0, point_array
	add	x2, x0, :lo12:point_array
	ldrsw	x3, [x29, 44]
	ldrsw	x1, [x29, 40]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	str	wzr, [x2, x0, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L640:
	ldr	w0, [x29, 44]
	cmp	w0, 9
	ble	.L641
	ldr	w0, [x29, 40]
	add	w0, w0, 1
	str	w0, [x29, 40]
.L639:
	ldr	w0, [x29, 40]
	cmp	w0, 29
	ble	.L642
	str	wzr, [x29, 40]
	b	.L643
.L646:
	str	wzr, [x29, 44]
	b	.L644
.L645:
	adrp	x0, pressure_array
	add	x2, x0, :lo12:pressure_array
	ldrsw	x3, [x29, 44]
	ldrsw	x1, [x29, 40]
	mov	x0, x1
	lsl	x0, x0, 2
	add	x0, x0, x1
	lsl	x0, x0, 1
	add	x0, x0, x3
	str	wzr, [x2, x0, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L644:
	ldr	w0, [x29, 44]
	cmp	w0, 9
	ble	.L645
	ldr	w0, [x29, 40]
	add	w0, w0, 1
	str	w0, [x29, 40]
.L643:
	ldr	w0, [x29, 40]
	cmp	w0, 7
	ble	.L646
	str	wzr, [x29, 44]
	b	.L647
.L648:
	adrp	x0, point_delay
	add	x0, x0, :lo12:point_delay
	ldrsw	x1, [x29, 44]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, filter_deep
	add	x0, x0, :lo12:filter_deep
	ldrsw	x1, [x29, 44]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, point_decimal
	add	x0, x0, :lo12:point_decimal
	ldrsw	x1, [x29, 44]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L647:
	ldr	w0, [x29, 44]
	cmp	w0, 9
	ble	.L648
	adrp	x0, point_edge
	add	x0, x0, :lo12:point_edge
	str	wzr, [x0]
	adrp	x0, point_n
	add	x0, x0, :lo12:point_n
	str	wzr, [x0]
	ldr	w0, [x29, 28]
	cmp	w0, wzr
	beq	.L649
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	wzr, [x0]
.L649:
	adrp	x0, prev_num
	add	x0, x0, :lo12:prev_num
	str	wzr, [x0]
	adrp	x0, point_shake
	add	x0, x0, :lo12:point_shake
	str	wzr, [x0]
	adrp	x0, reset_mask_send
	add	x0, x0, :lo12:reset_mask_send
	str	wzr, [x0]
	adrp	x0, reset_mask_max
	add	x0, x0, :lo12:reset_mask_max
	str	wzr, [x0]
	adrp	x0, reset_mask_count
	add	x0, x0, :lo12:reset_mask_count
	str	wzr, [x0]
	adrp	x0, point_near
	add	x0, x0, :lo12:point_near
	str	wzr, [x0]
	adrp	x0, point_corner
	add	x0, x0, :lo12:point_corner
	str	wzr, [x0]
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	str	wzr, [x0]
	adrp	x0, inte_count
	add	x0, x0, :lo12:inte_count
	str	wzr, [x0]
	adrp	x0, csensor_count
	add	x0, x0, :lo12:csensor_count
	str	wzr, [x0]
	adrp	x0, double_click
	add	x0, x0, :lo12:double_click
	str	wzr, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	wzr, [x0]
	bl	GestureInit
	ldp	x29, x30, [sp], 48
	ret
	.size	gsl_id_reg_init, .-gsl_id_reg_init
	.align	2
	.type	DataCheck, %function
DataCheck:
	adrp	x0, drv_num
	add	x0, x0, :lo12:drv_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L651
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L651
	adrp	x0, sen_num
	add	x0, x0, :lo12:sen_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L651
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L652
.L651:
	mov	w0, 0
	b	.L653
.L652:
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L654
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L655
.L654:
	mov	w0, 0
	b	.L653
.L655:
	mov	w0, 1
.L653:
	ret
	.size	DataCheck, .-DataCheck
	.align	2
	.global	gsl_DataInit
	.type	gsl_DataInit, %function
gsl_DataInit:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	str	x0, [x29, 40]
	mov	w0, 1
	bl	gsl_id_reg_init
	str	wzr, [x29, 76]
	b	.L657
.L658:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L657:
	ldr	w0, [x29, 76]
	cmp	w0, 9
	ble	.L658
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	str	x0, [x29, 56]
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	str	wzr, [x0]
	str	wzr, [x29, 76]
	b	.L659
.L660:
	ldr	w2, [x29, 76]
	adrp	x0, coordinate_correct_coe_x
	add	x0, x0, :lo12:coordinate_correct_coe_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w2, [x29, 76]
	adrp	x0, coordinate_correct_coe_y
	add	x0, x0, :lo12:coordinate_correct_coe_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L659:
	ldr	w0, [x29, 76]
	cmp	w0, 31
	ble	.L660
	adrp	x0, id_first_coe
	add	x0, x0, :lo12:id_first_coe
	mov	w1, 8
	str	w1, [x0]
	adrp	x0, id_speed_coe
	add	x0, x0, :lo12:id_speed_coe
	mov	w1, 16384
	str	w1, [x0]
	adrp	x0, id_static_coe
	add	x0, x0, :lo12:id_static_coe
	mov	w1, 4096
	str	w1, [x0]
	adrp	x0, average
	add	x0, x0, :lo12:average
	mov	w1, 4
	str	w1, [x0]
	adrp	x0, soft_average
	add	x0, x0, :lo12:soft_average
	mov	w1, 3
	str	w1, [x0]
	adrp	x0, report_delay
	add	x0, x0, :lo12:report_delay
	str	wzr, [x0]
	adrp	x0, delay_key
	add	x0, x0, :lo12:delay_key
	str	wzr, [x0]
	adrp	x0, report_ahead
	add	x1, x0, :lo12:report_ahead
	mov	w0, 37449
	movk	w0, 0x924, lsl 16
	str	w0, [x1]
	adrp	x0, report_delete
	add	x0, x0, :lo12:report_delete
	str	wzr, [x0]
	str	wzr, [x29, 76]
	b	.L661
.L662:
	adrp	x0, median_dis
	add	x1, x0, :lo12:median_dis
	ldrsw	x0, [x29, 76]
	strb	wzr, [x1, x0]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L661:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L662
	adrp	x0, shake_min
	add	x0, x0, :lo12:shake_min
	str	wzr, [x0]
	str	wzr, [x29, 76]
	b	.L663
.L664:
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L663:
	ldr	w0, [x29, 76]
	cmp	w0, 1
	ble	.L664
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	mov	w1, 4096
	str	w1, [x0]
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	mov	w1, 4096
	str	w1, [x0]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	mov	w1, 480
	str	w1, [x0]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	mov	w1, 800
	str	w1, [x0]
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	mov	w1, 10
	str	w1, [x0]
	adrp	x0, drv_num
	add	x0, x0, :lo12:drv_num
	mov	w1, 16
	str	w1, [x0]
	adrp	x0, sen_num
	add	x0, x0, :lo12:sen_num
	mov	w1, 10
	str	w1, [x0]
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	mov	w1, 16
	str	w1, [x0]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	mov	w1, 10
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L665
.L666:
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L665:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L666
	str	wzr, [x29, 76]
	b	.L667
.L668:
	adrp	x0, stretch_array
	add	x0, x0, :lo12:stretch_array
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L667:
	ldr	w0, [x29, 76]
	cmp	w0, 31
	ble	.L668
	str	wzr, [x29, 76]
	b	.L669
.L670:
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L669:
	ldr	w0, [x29, 76]
	cmp	w0, 15
	ble	.L670
	adrp	x0, reset_mask_dis
	add	x0, x0, :lo12:reset_mask_dis
	str	wzr, [x0]
	adrp	x0, reset_mask_type
	add	x0, x0, :lo12:reset_mask_type
	str	wzr, [x0]
	adrp	x0, edge_start
	add	x0, x0, :lo12:edge_start
	str	wzr, [x0]
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	str	wzr, [x0]
	adrp	x0, point_extend
	add	x0, x0, :lo12:point_extend
	str	wzr, [x0]
	adrp	x0, key_map_able
	add	x0, x0, :lo12:key_map_able
	str	wzr, [x0]
	str	wzr, [x29, 76]
	b	.L671
.L672:
	adrp	x0, key_range_array
	add	x0, x0, :lo12:key_range_array
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L671:
	ldr	w0, [x29, 76]
	cmp	w0, 23
	ble	.L672
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	str	wzr, [x0]
	adrp	x0, filter_coe
	add	x0, x0, :lo12:filter_coe
	mov	w1, 2584
	str	w1, [x0]
	adrp	x0, filter_coe
	add	x1, x0, :lo12:filter_coe
	mov	w0, 1560
	movk	w0, 0x1, lsl 16
	str	w0, [x1, 4]
	adrp	x0, filter_coe
	add	x1, x0, :lo12:filter_coe
	mov	w0, 33800
	movk	w0, 0x41, lsl 16
	str	w0, [x1, 8]
	adrp	x0, filter_coe
	add	x1, x0, :lo12:filter_coe
	mov	w0, 33542
	movk	w0, 0x641, lsl 16
	str	w0, [x1, 12]
	str	wzr, [x29, 76]
	b	.L673
.L674:
	adrp	x0, multi_x_array
	add	x0, x0, :lo12:multi_x_array
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	adrp	x0, multi_y_array
	add	x0, x0, :lo12:multi_y_array
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L673:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L674
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	mov	w1, 32
	str	w1, [x0]
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	mov	w1, 96
	str	w1, [x0, 4]
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	str	wzr, [x0]
	adrp	x0, edge_first_coe
	add	x0, x0, :lo12:edge_first_coe
	mov	w1, 128
	str	w1, [x0]
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	str	wzr, [x0]
	ldr	x0, [x29, 40]
	cmp	x0, xzr
	bne	.L675
	b	.L656
.L675:
	ldr	x0, [x29, 40]
	ldr	w0, [x0]
	cmp	w0, 4095
	bhi	.L677
	ldr	x0, [x29, 40]
	bl	ConfigCoorMulti
	cmp	w0, wzr
	beq	.L678
	mov	w0, 512
	str	w0, [x29, 68]
	b	.L682
.L678:
	ldr	x0, [x29, 40]
	bl	ConfigFilter
	cmp	w0, wzr
	beq	.L680
	mov	w0, 256
	str	w0, [x29, 68]
	b	.L682
.L680:
	ldr	x0, [x29, 40]
	bl	ConfigKeyMap
	cmp	w0, wzr
	beq	.L681
	mov	w0, 241
	str	w0, [x29, 68]
	b	.L682
.L681:
	mov	w0, 215
	str	w0, [x29, 68]
	b	.L682
.L677:
	ldr	x0, [x29, 40]
	add	x0, x0, 4
	ldr	w0, [x0]
	cmp	w0, 512
	bhi	.L683
	ldr	x0, [x29, 40]
	add	x0, x0, 4
	ldr	w0, [x0]
	str	w0, [x29, 68]
	b	.L682
.L683:
	mov	w0, 512
	str	w0, [x29, 68]
.L682:
	str	wzr, [x29, 76]
	b	.L684
.L685:
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldrsw	x1, [x29, 76]
	lsl	x1, x1, 2
	ldr	x2, [x29, 40]
	add	x1, x2, x1
	ldr	w1, [x1]
	str	w1, [x0]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L684:
	ldr	w1, [x29, 76]
	ldr	w0, [x29, 68]
	cmp	w1, w0
	blt	.L685
	b	.L686
.L687:
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	str	wzr, [x0]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L686:
	ldr	w0, [x29, 76]
	cmp	w0, 511
	ble	.L687
	ldr	x0, [x29, 40]
	ldr	w0, [x0]
	cmp	w0, 4095
	bhi	.L688
	ldr	x0, [x29, 56]
	ldr	w1, [x0]
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 4]
	adrp	x0, drv_num
	add	x0, x0, :lo12:drv_num
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 8]
	adrp	x0, sen_num
	add	x0, x0, :lo12:sen_num
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 12]
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 16]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 20]
	adrp	x0, id_first_coe
	add	x0, x0, :lo12:id_first_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 24]
	adrp	x0, id_speed_coe
	add	x0, x0, :lo12:id_speed_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 28]
	adrp	x0, id_static_coe
	add	x0, x0, :lo12:id_static_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 32]
	adrp	x0, average
	add	x0, x0, :lo12:average
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 36]
	adrp	x0, soft_average
	add	x0, x0, :lo12:soft_average
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 52]
	adrp	x0, report_delay
	add	x0, x0, :lo12:report_delay
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 56]
	adrp	x0, shake_min
	add	x0, x0, :lo12:shake_min
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 60
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 64
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 68
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 72]
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L689
.L690:
	ldr	w0, [x29, 76]
	add	w0, w0, 19
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	uxtb	w2, w0
	adrp	x0, median_dis
	add	x1, x0, :lo12:median_dis
	ldrsw	x0, [x29, 76]
	strb	w2, [x1, x0]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L689:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L690
	str	wzr, [x29, 76]
	b	.L691
.L692:
	ldr	w0, [x29, 76]
	add	w0, w0, 23
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 25
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 27
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 29
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L691:
	ldr	w0, [x29, 76]
	cmp	w0, 1
	ble	.L692
	str	wzr, [x29, 76]
	b	.L693
.L694:
	ldr	w0, [x29, 76]
	add	w0, w0, 31
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, coordinate_correct_coe_x
	add	x0, x0, :lo12:coordinate_correct_coe_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 95
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, coordinate_correct_coe_y
	add	x0, x0, :lo12:coordinate_correct_coe_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L693:
	ldr	w0, [x29, 76]
	cmp	w0, 63
	ble	.L694
	str	wzr, [x29, 76]
	b	.L695
.L696:
	ldr	w0, [x29, 76]
	add	w0, w0, 159
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L695:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L696
	str	wzr, [x29, 76]
	b	.L697
.L698:
	ldr	w0, [x29, 76]
	add	w0, w0, 163
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, stretch_array
	add	x0, x0, :lo12:stretch_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L697:
	ldr	w0, [x29, 76]
	cmp	w0, 31
	ble	.L698
	str	wzr, [x29, 76]
	b	.L699
.L700:
	ldr	w0, [x29, 76]
	add	w0, w0, 195
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L699:
	ldr	w0, [x29, 76]
	cmp	w0, 15
	ble	.L700
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 852]
	adrp	x0, reset_mask_dis
	add	x0, x0, :lo12:reset_mask_dis
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 856]
	adrp	x0, reset_mask_type
	add	x0, x0, :lo12:reset_mask_type
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 864]
	adrp	x0, edge_start
	add	x0, x0, :lo12:edge_start
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 868]
	adrp	x0, key_map_able
	add	x0, x0, :lo12:key_map_able
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L701
.L702:
	ldr	w0, [x29, 76]
	add	w0, w0, 218
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, key_range_array
	add	x0, x0, :lo12:key_range_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L701:
	ldr	w0, [x29, 76]
	cmp	w0, 23
	ble	.L702
	ldr	x0, [x29, 56]
	add	x0, x0, 968
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L703
.L704:
	ldr	w0, [x29, 76]
	add	w0, w0, 243
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, filter_coe
	add	x0, x0, :lo12:filter_coe
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L703:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L704
	str	wzr, [x29, 76]
	b	.L705
.L706:
	ldr	w0, [x29, 76]
	add	w0, w0, 247
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, multi_x_array
	add	x0, x0, :lo12:multi_x_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L705:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L706
	str	wzr, [x29, 76]
	b	.L707
.L708:
	ldr	w0, [x29, 76]
	add	w0, w0, 251
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, multi_y_array
	add	x0, x0, :lo12:multi_y_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L707:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L708
	ldr	x0, [x29, 56]
	add	x0, x0, 1020
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	str	w1, [x0]
	str	wzr, [x29, 72]
	b	.L709
.L712:
	str	wzr, [x29, 76]
	b	.L710
.L711:
	ldr	w0, [x29, 76]
	add	w1, w0, 256
	ldr	w0, [x29, 72]
	lsl	w0, w0, 6
	add	w0, w1, w0
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, multi_group
	add	x0, x0, :lo12:multi_group
	ldrsw	x1, [x29, 76]
	ldrsw	x3, [x29, 72]
	lsl	x3, x3, 6
	add	x1, x3, x1
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L710:
	ldr	w0, [x29, 76]
	cmp	w0, 63
	ble	.L711
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L709:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L712
	str	wzr, [x29, 72]
	b	.L713
.L716:
	str	wzr, [x29, 76]
	b	.L714
.L715:
	ldr	w0, [x29, 76]
	add	w1, w0, 448
	ldr	w0, [x29, 72]
	lsl	w0, w0, 3
	add	w0, w1, w0
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w3, w0
	adrp	x0, ps_coe
	add	x0, x0, :lo12:ps_coe
	ldrsw	x1, [x29, 76]
	ldrsw	x2, [x29, 72]
	lsl	x2, x2, 3
	add	x1, x2, x1
	str	w3, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w1, w0, 448
	ldr	w0, [x29, 72]
	lsl	w0, w0, 3
	add	w0, w1, w0
	sxtw	x0, w0
	add	x0, x0, 32
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w3, w0
	adrp	x0, pr_coe
	add	x0, x0, :lo12:pr_coe
	ldrsw	x1, [x29, 76]
	ldrsw	x2, [x29, 72]
	lsl	x2, x2, 3
	add	x1, x2, x1
	str	w3, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L714:
	ldr	w0, [x29, 76]
	cmp	w0, 7
	ble	.L715
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L713:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L716
	adrp	x0, near_set
	add	x0, x0, :lo12:near_set
	str	wzr, [x0]
	adrp	x0, near_set
	add	x0, x0, :lo12:near_set
	str	wzr, [x0, 4]
	b	.L717
.L688:
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 64]
	adrp	x0, global_flag
	add	x0, x0, :lo12:global_flag
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 68
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, point_num_max
	add	x0, x0, :lo12:point_num_max
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 72
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, drv_num
	add	x0, x0, :lo12:drv_num
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 72
	ldr	w0, [x0]
	lsr	w1, w0, 16
	adrp	x0, sen_num
	add	x0, x0, :lo12:sen_num
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 76
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 76
	ldr	w0, [x0]
	lsr	w1, w0, 16
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 80
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 80
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 84]
	adrp	x0, average
	add	x0, x0, :lo12:average
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 88]
	adrp	x0, reset_mask_dis
	add	x0, x0, :lo12:reset_mask_dis
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 92]
	adrp	x0, reset_mask_type
	add	x0, x0, :lo12:reset_mask_type
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 96
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 96
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, point_repeat
	add	x0, x0, :lo12:point_repeat
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 100
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, near_set
	add	x0, x0, :lo12:near_set
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 100
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, near_set
	add	x0, x0, :lo12:near_set
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 104
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, diagonal
	add	x0, x0, :lo12:diagonal
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 108
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, point_extend
	add	x0, x0, :lo12:point_extend
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 112]
	adrp	x0, edge_start
	add	x0, x0, :lo12:edge_start
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 124]
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 128]
	adrp	x0, id_first_coe
	add	x0, x0, :lo12:id_first_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 132]
	adrp	x0, id_speed_coe
	add	x0, x0, :lo12:id_speed_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 136]
	adrp	x0, id_static_coe
	add	x0, x0, :lo12:id_static_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 140
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 140
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 144
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 144
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 148
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 148
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 152
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 152
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 156
	ldr	w0, [x0]
	lsr	w1, w0, 24
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	str	w1, [x0]
	ldr	x0, [x29, 56]
	add	x0, x0, 156
	ldr	w0, [x0]
	lsr	w0, w0, 16
	and	w1, w0, 255
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	str	w1, [x0, 4]
	ldr	x0, [x29, 56]
	add	x0, x0, 156
	ldr	w0, [x0]
	lsr	w0, w0, 8
	and	w1, w0, 255
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	str	w1, [x0, 8]
	ldr	x0, [x29, 56]
	add	x0, x0, 156
	ldr	w0, [x0]
	and	w1, w0, 255
	adrp	x0, edge_cut
	add	x0, x0, :lo12:edge_cut
	str	w1, [x0, 12]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 160]
	adrp	x0, report_delay
	add	x0, x0, :lo12:report_delay
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 164]
	adrp	x0, shake_min
	add	x0, x0, :lo12:shake_min
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L718
.L719:
	ldr	w0, [x29, 76]
	lsl	w3, w0, 1
	ldr	w0, [x29, 76]
	add	w0, w0, 42
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w2, w0, 65535
	adrp	x0, stretch_array
	add	x0, x0, :lo12:stretch_array
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	lsl	w0, w0, 1
	add	w3, w0, 1
	ldr	w0, [x29, 76]
	add	w0, w0, 42
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	lsr	w2, w0, 16
	adrp	x0, stretch_array
	add	x0, x0, :lo12:stretch_array
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L718:
	ldr	w0, [x29, 76]
	cmp	w0, 15
	ble	.L719
	str	wzr, [x29, 76]
	b	.L720
.L721:
	ldr	w0, [x29, 76]
	lsl	w3, w0, 1
	ldr	w0, [x29, 76]
	add	w0, w0, 58
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w2, w0, 65535
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	lsl	w0, w0, 1
	add	w3, w0, 1
	ldr	w0, [x29, 76]
	add	w0, w0, 58
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	lsr	w2, w0, 16
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L720:
	ldr	w0, [x29, 76]
	cmp	w0, 7
	ble	.L721
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 264]
	adrp	x0, report_ahead
	add	x0, x0, :lo12:report_ahead
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 288]
	adrp	x0, edge_first
	add	x0, x0, :lo12:edge_first
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 292]
	adrp	x0, edge_first_coe
	add	x0, x0, :lo12:edge_first_coe
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 296]
	adrp	x0, delay_key
	add	x0, x0, :lo12:delay_key
	str	w1, [x0]
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 300]
	adrp	x0, report_delete
	add	x0, x0, :lo12:report_delete
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L722
.L723:
	ldr	w0, [x29, 76]
	lsl	w3, w0, 1
	ldr	w0, [x29, 76]
	add	w0, w0, 80
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w2, w0, 65535
	adrp	x0, stretch_active
	add	x0, x0, :lo12:stretch_active
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	lsl	w0, w0, 1
	add	w3, w0, 1
	ldr	w0, [x29, 76]
	add	w0, w0, 80
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	lsr	w2, w0, 16
	adrp	x0, stretch_active
	add	x0, x0, :lo12:stretch_active
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L722:
	ldr	w0, [x29, 76]
	cmp	w0, 15
	ble	.L723
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 384]
	adrp	x0, key_map_able
	add	x0, x0, :lo12:key_map_able
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L724
.L725:
	ldr	w0, [x29, 76]
	add	w0, w0, 97
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, key_range_array
	add	x0, x0, :lo12:key_range_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L724:
	ldr	w0, [x29, 76]
	cmp	w0, 23
	ble	.L725
	ldr	x0, [x29, 56]
	ldr	w1, [x0, 1024]
	adrp	x0, coordinate_correct_able
	add	x0, x0, :lo12:coordinate_correct_able
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L726
.L727:
	ldr	w0, [x29, 76]
	add	w0, w0, 257
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, multi_x_array
	add	x0, x0, :lo12:multi_x_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 261
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, multi_y_array
	add	x0, x0, :lo12:multi_y_array
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L726:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L727
	str	wzr, [x29, 76]
	b	.L728
.L729:
	ldr	w0, [x29, 76]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	add	w0, w0, 265
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	ldr	w1, [x29, 76]
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	adrp	x0, coordinate_correct_coe_x
	add	x0, x0, :lo12:coordinate_correct_coe_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	add	w0, w0, 281
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	ldr	w1, [x29, 76]
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	adrp	x0, coordinate_correct_coe_y
	add	x0, x0, :lo12:coordinate_correct_coe_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L728:
	ldr	w0, [x29, 76]
	cmp	w0, 63
	ble	.L729
	str	wzr, [x29, 72]
	b	.L730
.L733:
	str	wzr, [x29, 76]
	b	.L731
.L732:
	ldr	w0, [x29, 72]
	lsl	w1, w0, 6
	ldr	w0, [x29, 76]
	add	w0, w1, w0
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	add	w0, w0, 297
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	ldr	w0, [x29, 72]
	lsl	w1, w0, 6
	ldr	w0, [x29, 76]
	add	w1, w1, w0
	asr	w0, w1, 31
	lsr	w0, w0, 30
	add	w1, w1, w0
	and	w1, w1, 3
	sub	w0, w1, w0
	lsl	w0, w0, 3
	lsr	w0, w2, w0
	and	w2, w0, 255
	adrp	x0, multi_group
	add	x0, x0, :lo12:multi_group
	ldrsw	x1, [x29, 76]
	ldrsw	x3, [x29, 72]
	lsl	x3, x3, 6
	add	x1, x3, x1
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L731:
	ldr	w0, [x29, 76]
	cmp	w0, 63
	ble	.L732
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L730:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L733
	ldr	x0, [x29, 56]
	add	x0, x0, 1536
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, filter_able
	add	x0, x0, :lo12:filter_able
	str	w1, [x0]
	str	wzr, [x29, 76]
	b	.L734
.L735:
	ldr	w0, [x29, 76]
	add	w0, w0, 385
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w2, [x0]
	adrp	x0, filter_coe
	add	x0, x0, :lo12:filter_coe
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L734:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L735
	str	wzr, [x29, 76]
	b	.L736
.L737:
	ldr	w0, [x29, 76]
	add	w0, w0, 389
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	uxtb	w2, w0
	adrp	x0, median_dis
	add	x1, x0, :lo12:median_dis
	ldrsw	x0, [x29, 76]
	strb	w2, [x1, x0]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L736:
	ldr	w0, [x29, 76]
	cmp	w0, 3
	ble	.L737
	str	wzr, [x29, 72]
	b	.L738
.L741:
	str	wzr, [x29, 76]
	b	.L739
.L740:
	ldr	w0, [x29, 76]
	add	w1, w0, 393
	ldr	w0, [x29, 72]
	lsl	w0, w0, 3
	add	w0, w1, w0
	sxtw	x0, w0
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w3, w0
	adrp	x0, ps_coe
	add	x0, x0, :lo12:ps_coe
	ldrsw	x1, [x29, 76]
	ldrsw	x2, [x29, 72]
	lsl	x2, x2, 3
	add	x1, x2, x1
	str	w3, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w1, w0, 393
	ldr	w0, [x29, 72]
	lsl	w0, w0, 3
	add	w0, w1, w0
	sxtw	x0, w0
	add	x0, x0, 32
	lsl	x0, x0, 2
	ldr	x1, [x29, 56]
	add	x0, x1, x0
	ldr	w0, [x0]
	mov	w3, w0
	adrp	x0, pr_coe
	add	x0, x0, :lo12:pr_coe
	ldrsw	x1, [x29, 76]
	ldrsw	x2, [x29, 72]
	lsl	x2, x2, 3
	add	x1, x2, x1
	str	w3, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L739:
	ldr	w0, [x29, 76]
	cmp	w0, 7
	ble	.L740
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L738:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L741
	ldr	x0, [x29, 56]
	add	x0, x0, 1828
	bl	GestureSet
.L717:
	adrp	x0, average
	add	x0, x0, :lo12:average
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L742
	adrp	x0, average
	add	x0, x0, :lo12:average
	mov	w1, 4
	str	w1, [x0]
.L742:
	str	wzr, [x29, 76]
	b	.L743
.L746:
	ldr	w0, [x29, 76]
	lsl	w1, w0, 1
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w1
	ldr	w0, [x0, x1, lsl 2]
	and	w0, w0, 32768
	cmp	w0, wzr
	beq	.L744
	ldr	w0, [x29, 76]
	lsl	w3, w0, 1
	ldr	w0, [x29, 76]
	lsl	w1, w0, 1
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w1
	ldr	w0, [x0, x1, lsl 2]
	and	w2, w0, -32769
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w3
	str	w2, [x0, x1, lsl 2]
	b	.L745
.L744:
	ldr	w0, [x29, 76]
	lsl	w19, w0, 1
	ldr	w0, [x29, 76]
	lsl	w1, w0, 1
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w1
	ldr	w0, [x0, x1, lsl 2]
	bl	Sqrt
	mov	w2, w0
	adrp	x0, shake_all_array
	add	x0, x0, :lo12:shake_all_array
	sxtw	x1, w19
	str	w2, [x0, x1, lsl 2]
.L745:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L743:
	ldr	w0, [x29, 76]
	cmp	w0, 7
	ble	.L746
	str	wzr, [x29, 76]
	b	.L747
.L752:
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	and	w0, w0, 32768
	cmp	w0, wzr
	beq	.L748
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	orr	w0, w0, -65536
	mov	w2, w0
	adrp	x0, match_x
	add	x0, x0, :lo12:match_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
.L748:
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	and	w0, w0, 32768
	cmp	w0, wzr
	beq	.L749
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	orr	w0, w0, -65536
	mov	w2, w0
	adrp	x0, match_y
	add	x0, x0, :lo12:match_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
.L749:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	and	w0, w0, 32768
	cmp	w0, wzr
	beq	.L750
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	orr	w0, w0, -65536
	mov	w2, w0
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
.L750:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	and	w0, w0, 32768
	cmp	w0, wzr
	beq	.L751
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldrsw	x1, [x29, 76]
	ldr	w0, [x0, x1, lsl 2]
	orr	w0, w0, -65536
	mov	w2, w0
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldrsw	x1, [x29, 76]
	str	w2, [x0, x1, lsl 2]
.L751:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L747:
	ldr	w0, [x29, 76]
	cmp	w0, 1
	ble	.L752
	str	wzr, [x29, 76]
	b	.L753
.L754:
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	ldrsw	x1, [x29, 76]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L753:
	ldr	w0, [x29, 76]
	cmp	w0, 511
	ble	.L754
.L656:
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 80
	ret
	.size	gsl_DataInit, .-gsl_DataInit
	.align	2
	.global	gsl_version_id
	.type	gsl_version_id, %function
gsl_version_id:
	mov	w0, 517
	movk	w0, 0x2015, lsl 16
	ret
	.size	gsl_version_id, .-gsl_version_id
	.align	2
	.global	gsl_mask_tiaoping
	.type	gsl_mask_tiaoping, %function
gsl_mask_tiaoping:
	adrp	x0, reset_mask_send
	add	x0, x0, :lo12:reset_mask_send
	ldr	w0, [x0]
	ret
	.size	gsl_mask_tiaoping, .-gsl_mask_tiaoping
	.align	2
	.type	GetFlag, %function
GetFlag:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	wzr, [x29, 28]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 32768
	cmp	w0, wzr
	beq	.L760
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w1, w0, -16777216
	mov	w0, 1509949440
	cmp	w1, w0
	bne	.L760
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	asr	w0, w0, 16
	and	w1, w0, 255
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
.L760:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 256
	cmp	w0, wzr
	bne	.L761
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 512
	cmp	w0, wzr
	beq	.L762
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, -128
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L762
.L761:
	mov	w0, 0
	bl	gsl_id_reg_init
.L762:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 768
	cmp	w0, wzr
	bne	.L763
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	orr	w1, w1, -128
	strb	w1, [x0, 3]
.L763:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 1024
	cmp	w0, wzr
	beq	.L764
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	orr	w1, w1, 32
	strb	w1, [x0, 3]
	b	.L765
.L764:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	and	w1, w1, -33
	strb	w1, [x0, 3]
.L765:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 8192
	cmp	w0, wzr
	beq	.L766
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	orr	w1, w1, 15
	strb	w1, [x0, 3]
	b	.L767
.L766:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	and	w0, w0, 15
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L767
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w0, [x0, 3]
	ubfx	x0, x0, 0, 4
	uxtb	w0, w0
	add	w0, w0, 15
	and	w0, w0, 15
	uxtb	w2, w0
	adrp	x0, global_state
	add	x1, x0, :lo12:global_state
	ldrb	w0, [x1, 3]
	bfi	w0, w2, 0, 4
	strb	w0, [x1, 3]
.L767:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 16384
	cmp	w0, wzr
	beq	.L768
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 2]
	orr	w1, w1, -128
	strb	w1, [x0, 2]
	b	.L769
.L768:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 2]
	and	w1, w1, 127
	strb	w1, [x0, 2]
.L769:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 255
	cmp	w0, wzr
	beq	.L770
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 2048
	cmp	w0, wzr
	beq	.L771
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	orr	w1, w1, 16
	strb	w1, [x0, 3]
	b	.L770
.L771:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	and	w1, w1, -17
	strb	w1, [x0, 3]
.L770:
	adrp	x0, inte_count
	add	x0, x0, :lo12:inte_count
	ldr	w0, [x0]
	add	w1, w0, 1
	adrp	x0, inte_count
	add	x0, x0, :lo12:inte_count
	str	w1, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	lsr	w1, w0, 16
	adrp	x0, csensor_count
	add	x0, x0, :lo12:csensor_count
	str	w1, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 255
	str	w0, [x29, 24]
	ldr	w0, [x29, 24]
	cmp	w0, 10
	ble	.L772
	mov	w0, 10
	str	w0, [x29, 24]
.L772:
	str	wzr, [x29, 28]
	b	.L773
.L775:
	ldr	w1, [x29, 28]
	ldr	w0, [x29, 24]
	cmp	w1, w0
	blt	.L774
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 28]
	str	wzr, [x0, x1, lsl 2]
.L774:
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L773:
	ldr	w0, [x29, 28]
	cmp	w0, 9
	ble	.L775
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w1, w0, -256
	ldr	w0, [x29, 24]
	add	w1, w1, w0
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	w1, [x0]
	ldp	x29, x30, [sp], 32
	ret
	.size	GetFlag, .-GetFlag
	.align	2
	.type	PointIgnore, %function
PointIgnore:
	sub	sp, sp, #16
	adrp	x0, id_flag
	add	x0, x0, :lo12:id_flag
	ldrb	w0, [x0]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	bne	.L777
	b	.L776
.L777:
	str	wzr, [sp, 12]
	b	.L779
.L789:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrb	w0, [x0, 1]
	ubfx	x0, x0, 4, 1
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L780
	b	.L781
.L780:
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	str	w0, [sp, 4]
	adrp	x0, point_now
	add	x1, x0, :lo12:point_now
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	mul	w1, w1, w0
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w0, w0, 6
	sdiv	w0, w1, w0
	str	w0, [sp, 8]
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L782
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	beq	.L783
.L782:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0]
	ldr	w0, [sp, 4]
	cmp	w1, w0
	ble	.L784
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L784:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0, 4]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	bgt	.L785
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w1, [x0]
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w0, [x0, 4]
	sub	w1, w1, w0
	ldr	w0, [sp, 4]
	cmp	w1, w0
	bge	.L785
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L785:
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0, 4]
	adrp	x0, screen_y_max
	add	x0, x0, :lo12:screen_y_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	blt	.L783
	adrp	x0, ignore_y
	add	x0, x0, :lo12:ignore_y
	ldr	w1, [x0, 4]
	ldr	w0, [sp, 4]
	cmp	w1, w0
	bge	.L783
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L783:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L786
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	beq	.L781
.L786:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0]
	ldr	w0, [sp, 8]
	cmp	w1, w0
	ble	.L787
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L787:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0, 4]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	bgt	.L788
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w1, [x0]
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w0, [x0, 4]
	sub	w1, w1, w0
	ldr	w0, [sp, 8]
	cmp	w1, w0
	bge	.L788
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L788:
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0, 4]
	adrp	x0, screen_x_max
	add	x0, x0, :lo12:screen_x_max
	ldr	w0, [x0]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	cmp	w1, w0
	blt	.L781
	adrp	x0, ignore_x
	add	x0, x0, :lo12:ignore_x
	ldr	w1, [x0, 4]
	ldr	w0, [sp, 8]
	cmp	w1, w0
	bge	.L781
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	str	wzr, [x0, x1, lsl 2]
.L781:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L779:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [sp, 12]
	cmp	w1, w0
	blt	.L789
	str	wzr, [sp, 8]
	str	wzr, [sp, 12]
	b	.L790
.L793:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [sp, 12]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	bne	.L791
	b	.L792
.L791:
	ldr	w0, [sp, 8]
	add	w1, w0, 1
	str	w1, [sp, 8]
	adrp	x1, point_now
	add	x1, x1, :lo12:point_now
	sxtw	x0, w0
	adrp	x2, point_now
	add	x2, x2, :lo12:point_now
	ldrsw	x3, [sp, 12]
	ldr	w2, [x2, x3, lsl 2]
	str	w2, [x1, x0, lsl 2]
.L792:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L790:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	ldr	w1, [sp, 12]
	cmp	w1, w0
	blt	.L793
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w1, [sp, 8]
	str	w1, [x0]
.L776:
	add	sp, sp, 16
	ret
	.size	PointIgnore, .-PointIgnore
	.align	2
	.global	gsl_alg_id_main
	.type	gsl_alg_id_main, %function
gsl_alg_id_main:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x0, [x29, 24]
	ldr	x0, [x29, 24]
	ldr	w1, [x0, 120]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	w1, [x0]
	str	wzr, [x29, 44]
	b	.L795
.L796:
	ldr	x0, [x29, 24]
	ldrsw	x1, [x29, 44]
	add	x1, x1, 20
	ldr	w0, [x0, x1, lsl 2]
	lsl	w1, w0, 28
	ldr	x0, [x29, 24]
	ldrsw	x2, [x29, 44]
	ldr	w0, [x0, x2, lsl 2]
	lsl	w0, w0, 16
	orr	w1, w1, w0
	ldr	x2, [x29, 24]
	ldrsw	x0, [x29, 44]
	add	x0, x0, 8
	lsl	x0, x0, 2
	add	x0, x2, x0
	ldr	w0, [x0, 8]
	orr	w0, w1, w0
	mov	w2, w0
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrsw	x1, [x29, 44]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L795:
	ldr	w0, [x29, 44]
	cmp	w0, 9
	ble	.L796
	bl	GestureJoint
	cmp	w0, wzr
	beq	.L797
	ldr	x0, [x29, 24]
	str	wzr, [x0, 120]
	str	wzr, [x29, 44]
	b	.L798
.L799:
	ldr	x0, [x29, 24]
	ldrsw	x1, [x29, 44]
	add	x1, x1, 20
	str	wzr, [x0, x1, lsl 2]
	ldr	x0, [x29, 24]
	ldrsw	x1, [x29, 44]
	str	wzr, [x0, x1, lsl 2]
	ldr	x1, [x29, 24]
	ldrsw	x0, [x29, 44]
	add	x0, x0, 8
	lsl	x0, x0, 2
	add	x0, x1, x0
	str	wzr, [x0, 8]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L798:
	ldr	w0, [x29, 44]
	cmp	w0, 9
	ble	.L799
	b	.L794
.L797:
	bl	GetFlag
	bl	DataCheck
	cmp	w0, wzr
	bne	.L801
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	wzr, [x0]
	ldr	x0, [x29, 24]
	str	wzr, [x0, 120]
	b	.L794
.L801:
	bl	PressureSave
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w1, w0, 255
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	w1, [x0]
	bl	PointIgnore
	bl	PointCoor
	bl	CoordinateCorrect
	bl	PointEdge
	bl	PointRepeat
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	bl	GetPointNum
	bl	PointPointer
	bl	PointPredict
	bl	PointId
	bl	PointNewId
	bl	PointOrder
	bl	PointCross
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	bl	GetPointNum
	bl	DoubleClick
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w1, [x0]
	adrp	x0, prev_num
	add	x0, x0, :lo12:prev_num
	str	w1, [x0]
	bl	ResetMask
	bl	PointStretch
	bl	PointDiagonal
	bl	PointFilter
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0]
	bl	GetPointNum
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0]
	mov	x2, x0
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	mov	w1, w0
	mov	x0, x2
	bl	GestureMain
	bl	PointDelay
	bl	PointMenu
	bl	PointExtend
	bl	PointPressure
	ldr	x0, [x29, 24]
	bl	PointReport
.L794:
	ldp	x29, x30, [sp], 48
	ret
	.size	gsl_alg_id_main, .-gsl_alg_id_main
	.align	2
	.type	DoubleClick, %function
DoubleClick:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x0, double_click
	add	x0, x0, :lo12:double_click
	str	wzr, [x0]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, 1
	bgt	.L803
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, 1
	bne	.L804
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L803
.L804:
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldrb	w0, [x0, 1]
	and	w0, w0, 16
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L805
.L803:
	str	wzr, [x29, 44]
	b	.L806
.L807:
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	ldrsw	x1, [x29, 44]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L806:
	ldr	w0, [x29, 44]
	cmp	w0, 3
	bls	.L807
	b	.L802
.L805:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L809
	adrp	x0, prev_num
	add	x0, x0, :lo12:prev_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L809
	mov	w0, 3
	str	w0, [x29, 44]
	b	.L810
.L811:
	ldr	w0, [x29, 44]
	sub	w1, w0, #1
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	sxtw	x1, w1
	ldr	w2, [x0, x1, lsl 2]
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	ldrsw	x1, [x29, 44]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	sub	w0, w0, #1
	str	w0, [x29, 44]
.L810:
	ldr	w0, [x29, 44]
	cmp	w0, wzr
	bgt	.L811
	adrp	x0, csensor_count
	add	x0, x0, :lo12:csensor_count
	ldr	w1, [x0]
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	str	w1, [x0]
	mov	w0, 3
	str	w0, [x29, 44]
	b	.L812
.L813:
	ldr	w0, [x29, 44]
	sub	w1, w0, #2
	adrp	x0, point_click
	add	x0, x0, :lo12:point_click
	sxtw	x1, w1
	ldr	w2, [x0, x1, lsl 2]
	adrp	x0, point_click
	add	x0, x0, :lo12:point_click
	ldrsw	x1, [x29, 44]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	sub	w0, w0, #1
	str	w0, [x29, 44]
.L812:
	ldr	w0, [x29, 44]
	cmp	w0, 1
	bgt	.L813
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w1, [x0]
	adrp	x0, point_click
	add	x0, x0, :lo12:point_click
	str	w1, [x0]
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w1, [x0]
	adrp	x0, point_click
	add	x0, x0, :lo12:point_click
	str	w1, [x0, 4]
.L809:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L814
	adrp	x0, prev_num
	add	x0, x0, :lo12:prev_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L814
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x1, [x0]
	adrp	x0, point_click+4
	add	x0, x0, :lo12:point_click+4
	bl	PointDistance
	mov	w19, w0
	adrp	x0, point_click
	add	x1, x0, :lo12:point_click
	adrp	x0, point_click+4
	add	x0, x0, :lo12:point_click+4
	bl	PointDistance
	cmp	w19, w0
	bls	.L814
	adrp	x0, point_pointer
	add	x0, x0, :lo12:point_pointer
	ldr	x0, [x0]
	ldr	w1, [x0]
	adrp	x0, point_click
	add	x0, x0, :lo12:point_click
	str	w1, [x0]
.L814:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L802
	adrp	x0, prev_num
	add	x0, x0, :lo12:prev_num
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L802
	mov	w0, 3
	str	w0, [x29, 44]
	b	.L816
.L817:
	ldr	w0, [x29, 44]
	sub	w1, w0, #1
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	sxtw	x1, w1
	ldr	w2, [x0, x1, lsl 2]
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	ldrsw	x1, [x29, 44]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	sub	w0, w0, #1
	str	w0, [x29, 44]
.L816:
	ldr	w0, [x29, 44]
	cmp	w0, wzr
	bgt	.L817
	adrp	x0, csensor_count
	add	x0, x0, :lo12:csensor_count
	ldr	w1, [x0]
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	str	w1, [x0]
	str	wzr, [x29, 44]
	b	.L818
.L819:
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	ldrsw	x1, [x29, 44]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	add	w2, w0, 1
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	sxtw	x2, w2
	ldr	w0, [x0, x2, lsl 2]
	sub	w0, w1, w0
	and	w2, w0, 65535
	add	x0, x29, 32
	ldrsw	x1, [x29, 44]
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L818:
	ldr	w0, [x29, 44]
	cmp	w0, 2
	bls	.L819
	add	x0, x29, 32
	ldr	w1, [x0]
	adrp	x0, double_down
	add	x0, x0, :lo12:double_down
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, average
	add	x0, x0, :lo12:average
	ldr	w0, [x0]
	mul	w0, w2, w0
	cmp	w1, w0
	bcc	.L820
	add	x0, x29, 32
	ldr	w1, [x0, 8]
	adrp	x0, double_down
	add	x0, x0, :lo12:double_down
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, average
	add	x0, x0, :lo12:average
	ldr	w0, [x0]
	mul	w0, w2, w0
	cmp	w1, w0
	bcc	.L820
	add	x0, x29, 32
	ldr	w1, [x0, 4]
	adrp	x0, double_up
	add	x0, x0, :lo12:double_up
	ldr	w0, [x0]
	mov	w2, w0
	adrp	x0, average
	add	x0, x0, :lo12:average
	ldr	w0, [x0]
	mul	w0, w2, w0
	cmp	w1, w0
	bls	.L821
.L820:
	b	.L802
.L821:
	adrp	x0, point_click+4
	add	x1, x0, :lo12:point_click+4
	adrp	x0, point_click
	add	x0, x0, :lo12:point_click
	bl	PointDistance
	cmp	w0, 4096
	bhi	.L822
	adrp	x0, point_click+12
	add	x1, x0, :lo12:point_click+12
	adrp	x0, point_click+8
	add	x0, x0, :lo12:point_click+8
	bl	PointDistance
	cmp	w0, 4096
	bhi	.L822
	adrp	x0, point_click+12
	add	x1, x0, :lo12:point_click+12
	adrp	x0, point_click+4
	add	x0, x0, :lo12:point_click+4
	bl	PointDistance
	cmp	w0, 16384
	bls	.L823
.L822:
	b	.L802
.L823:
	str	wzr, [x29, 44]
	b	.L824
.L825:
	adrp	x0, click_count
	add	x0, x0, :lo12:click_count
	ldrsw	x1, [x29, 44]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L824:
	ldr	w0, [x29, 44]
	cmp	w0, 3
	bls	.L825
	adrp	x0, double_click
	add	x0, x0, :lo12:double_click
	mov	w1, 42
	str	w1, [x0]
.L802:
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
	.size	DoubleClick, .-DoubleClick
	.align	2
	.global	gsl_FunIICRead
	.type	gsl_FunIICRead, %function
gsl_FunIICRead:
	sub	sp, sp, #16
	str	x0, [sp, 8]
	adrp	x0, ReadIICInt
	add	x0, x0, :lo12:ReadIICInt
	ldr	x1, [sp, 8]
	str	x1, [x0]
	add	sp, sp, 16
	ret
	.size	gsl_FunIICRead, .-gsl_FunIICRead
	.align	2
	.type	GestureJoint, %function
GestureJoint:
	sub	sp, sp, #528
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	str	wzr, [x29, 540]
	str	wzr, [x29, 528]
	str	wzr, [x29, 532]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w0, w0, 32768
	cmp	w0, wzr
	bne	.L829
	mov	w0, 0
	b	.L849
.L829:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	and	w1, w0, -16777216
	mov	w0, 1526726656
	cmp	w1, w0
	beq	.L831
	mov	w0, 0
	b	.L849
.L831:
	adrp	x0, ReadIICInt
	add	x0, x0, :lo12:ReadIICInt
	ldr	x0, [x0]
	cmp	x0, xzr
	bne	.L832
	mov	w0, 0
	b	.L849
.L832:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldr	w0, [x0]
	str	w0, [x29, 528]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldr	w0, [x0, 4]
	str	w0, [x29, 540]
	ldr	w0, [x29, 540]
	cmp	w0, 128
	bls	.L833
	mov	w0, 128
	str	w0, [x29, 540]
.L833:
	adrp	x0, ReadIICInt
	add	x0, x0, :lo12:ReadIICInt
	ldr	x3, [x0]
	add	x0, x29, 16
	ldr	w2, [x29, 540]
	ldr	w1, [x29, 528]
	blr	x3
	ldr	w0, [x29, 540]
	cmp	w0, 7
	bhi	.L834
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	mov	w1, 2147483647
	str	w1, [x0]
	mov	w0, 1
	b	.L849
.L834:
	str	wzr, [x29, 536]
	b	.L835
.L836:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldr	w1, [x29, 536]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 536]
	add	w0, w0, 1
	str	w0, [x29, 536]
.L835:
	ldr	w0, [x29, 536]
	cmp	w0, 9
	bls	.L836
	bl	GestureInit
	str	wzr, [x29, 536]
	b	.L837
.L846:
	ldr	w1, [x29, 536]
	ldr	w0, [x29, 540]
	cmp	w1, w0
	bcs	.L838
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	mov	w1, 1
	str	w1, [x0]
	add	x0, x29, 16
	ldr	w1, [x29, 536]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	str	w1, [x0]
	add	x0, x29, 16
	ldr	w1, [x29, 536]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, wzr
	beq	.L839
	add	x0, x29, 16
	ldr	w1, [x29, 536]
	ldr	w1, [x0, x1, lsl 2]
	mov	w0, 36864
	movk	w0, 0xe000, lsl 16
	and	w0, w1, w0
	cmp	w0, wzr
	beq	.L840
.L839:
	bl	GestureInit
	b	.L841
.L840:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldrb	w0, [x0, 1]
	and	w0, w0, 32
	uxtb	w0, w0
	cmp	w0, wzr
	beq	.L842
	mov	w0, 8192
	str	w0, [x29, 532]
	b	.L843
.L842:
	str	wzr, [x29, 532]
.L843:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	orr	w1, w1, 32
	strb	w1, [x0, 3]
	b	.L844
.L838:
	adrp	x0, global_state
	add	x0, x0, :lo12:global_state
	ldrb	w1, [x0, 3]
	and	w1, w1, -33
	strb	w1, [x0, 3]
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	wzr, [x0]
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	str	wzr, [x0]
.L844:
	bl	PointCoor
	bl	CoordinateCorrect
	bl	PointEdge
	bl	PointPointer
	bl	PointPredict
	bl	PointId
	bl	PointNewId
	bl	PointOrder
	bl	PointStretch
	bl	PointFilter
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0]
	bl	GetPointNum
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0]
	adrp	x1, point_report
	add	x1, x1, :lo12:point_report
	ldr	x1, [x1]
	ldr	w2, [x1]
	ldr	w1, [x29, 532]
	add	w1, w2, w1
	str	w1, [x0]
	ldr	w0, [x29, 540]
	add	w1, w0, 2
	ldr	w0, [x29, 536]
	cmp	w1, w0
	bls	.L845
	adrp	x0, point_report
	add	x0, x0, :lo12:point_report
	ldr	x0, [x0]
	mov	x2, x0
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	ldr	w0, [x0]
	mov	w1, w0
	mov	x0, x2
	bl	GestureMain
.L845:
	ldr	w0, [x29, 536]
	add	w0, w0, 1
	str	w0, [x29, 536]
.L837:
	ldr	w0, [x29, 540]
	add	w1, w0, 5
	ldr	w0, [x29, 536]
	cmp	w1, w0
	bhi	.L846
.L841:
	adrp	x0, point_num
	add	x0, x0, :lo12:point_num
	str	wzr, [x0]
	str	wzr, [x29, 536]
	b	.L847
.L848:
	adrp	x0, point_now
	add	x0, x0, :lo12:point_now
	ldr	w1, [x29, 536]
	str	wzr, [x0, x1, lsl 2]
	ldr	w0, [x29, 536]
	add	w0, w0, 1
	str	w0, [x29, 536]
.L847:
	ldr	w0, [x29, 536]
	cmp	w0, 9
	bls	.L848
	mov	w0, 1
.L849:
	ldp	x29, x30, [sp], 16
	add	sp, sp, 528
	ret
	.size	GestureJoint, .-GestureJoint
	.align	2
	.global	gsl_obtain_gesture
	.type	gsl_obtain_gesture, %function
gsl_obtain_gesture:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	GestureDeal
	ldp	x29, x30, [sp], 16
	ret
	.size	gsl_obtain_gesture, .-gsl_obtain_gesture
	.align	2
	.type	GestureMain, %function
GestureMain:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x0, [x29, 24]
	str	w1, [x29, 20]
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	str	wzr, [x0]
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L853
	ldr	x0, [x29, 24]
	cmp	x0, xzr
	bne	.L854
.L853:
	mov	w0, 0
	b	.L867
.L854:
	ldr	x0, [x29, 24]
	bl	GestureMulti
	mov	w1, w0
	adrp	x0, gesture_multi
	add	x0, x0, :lo12:gesture_multi
	str	w1, [x0]
	adrp	x0, gesture_multi
	add	x0, x0, :lo12:gesture_multi
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L856
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	mov	w1, 2
	str	w1, [x0]
	mov	w0, 1
	b	.L867
.L856:
	ldr	w0, [x29, 20]
	cmp	w0, wzr
	bne	.L857
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L858
	mov	w0, 0
	b	.L867
.L858:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	cmp	w0, 8
	bgt	.L859
	bl	GestureInit
	mov	w0, 0
	b	.L867
.L859:
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	mov	w1, 2147483647
	str	w1, [x0]
	mov	w0, 1
	b	.L867
.L857:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bge	.L860
	mov	w0, 0
	b	.L867
.L860:
	ldr	w0, [x29, 20]
	cmp	w0, 1
	bne	.L861
	ldr	x0, [x29, 24]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L861
	ldr	x0, [x29, 24]
	bl	GesturePush
	mov	w0, 0
	b	.L867
.L861:
	ldr	w0, [x29, 20]
	cmp	w0, 2
	bne	.L862
	ldr	x0, [x29, 24]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L862
	ldr	x0, [x29, 24]
	add	x0, x0, 4
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L863
.L862:
	ldr	w0, [x29, 20]
	cmp	w0, 1
	bne	.L864
	ldr	x0, [x29, 24]
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L864
	ldr	x0, [x29, 24]
	add	x0, x0, 4
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L864
.L863:
	ldr	x0, [x29, 24]
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L865
	ldr	x0, [x29, 24]
	add	x0, x0, 4
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L865
	ldr	x0, [x29, 24]
	ldr	w0, [x0]
	and	w0, w0, -61441
	str	w0, [x29, 44]
	ldr	x0, [x29, 24]
	add	x0, x0, 4
	ldr	w0, [x0]
	and	w1, w0, -61441
	ldr	w0, [x29, 44]
	add	w0, w1, w0
	str	w0, [x29, 44]
	ldr	w0, [x29, 44]
	lsr	w0, w0, 1
	and	w0, w0, -61441
	str	w0, [x29, 44]
	b	.L866
.L865:
	ldr	x0, [x29, 24]
	add	x0, x0, 4
	ldr	w0, [x0]
	and	w0, w0, -61441
	str	w0, [x29, 44]
.L866:
	ldr	w0, [x29, 44]
	orr	w0, w0, 8192
	str	w0, [x29, 44]
	add	x0, x29, 44
	bl	GesturePush
	mov	w0, 0
	b	.L867
.L864:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	mov	w1, -1
	str	w1, [x0]
	mov	w0, 0
.L867:
	ldp	x29, x30, [sp], 48
	ret
	.size	GestureMain, .-GestureMain
	.align	2
	.type	GestureSqrt, %function
GestureSqrt:
	sub	sp, sp, #32
	str	w0, [sp, 12]
	str	wzr, [sp, 28]
	mov	w0, 14
	str	w0, [sp, 24]
	b	.L869
.L871:
	ldr	w0, [sp, 24]
	mov	w1, 1
	lsl	w1, w1, w0
	ldr	w0, [sp, 28]
	add	w1, w1, w0
	ldr	w0, [sp, 24]
	mov	w2, 1
	lsl	w2, w2, w0
	ldr	w0, [sp, 28]
	add	w0, w2, w0
	mul	w1, w1, w0
	ldr	w0, [sp, 12]
	cmp	w1, w0
	bgt	.L870
	ldr	w0, [sp, 24]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [sp, 28]
	orr	w0, w1, w0
	str	w0, [sp, 28]
.L870:
	ldr	w0, [sp, 24]
	sub	w0, w0, #1
	str	w0, [sp, 24]
.L869:
	ldr	w0, [sp, 24]
	cmp	w0, wzr
	bge	.L871
	ldr	w0, [sp, 28]
	add	sp, sp, 32
	ret
	.size	GestureSqrt, .-GestureSqrt
	.align	2
	.type	GestureDistance, %function
GestureDistance:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x0, [x29, 40]
	str	x1, [x29, 32]
	str	w2, [x29, 28]
	ldr	w0, [x29, 28]
	cmp	w0, wzr
	beq	.L874
	ldr	x0, [x29, 40]
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0, 2]
	sub	w1, w1, w0
	ldr	x0, [x29, 40]
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0, 2]
	sub	w0, w2, w0
	mul	w1, w1, w0
	ldr	x0, [x29, 40]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w2, w2, w0
	ldr	x0, [x29, 40]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w0, w3, w0
	mul	w0, w2, w0
	add	w0, w1, w0
	bl	GestureSqrt
	b	.L875
.L874:
	ldr	x0, [x29, 40]
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0, 2]
	sub	w1, w1, w0
	ldr	x0, [x29, 40]
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0, 2]
	sub	w0, w2, w0
	mul	w1, w1, w0
	ldr	x0, [x29, 40]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w2, w2, w0
	ldr	x0, [x29, 40]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldr	x0, [x29, 32]
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w0, w3, w0
	mul	w0, w2, w0
	add	w0, w1, w0
.L875:
	ldp	x29, x30, [sp], 48
	ret
	.size	GestureDistance, .-GestureDistance
	.align	2
	.type	GesturePush, %function
GesturePush:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x0, [x29, 24]
	ldr	x0, [x29, 24]
	ldr	w1, [x0]
	mov	w0, -53249
	and	w0, w1, w0
	str	w0, [x29, 44]
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	cmp	w0, 255
	ble	.L877
	mov	w0, 0
	b	.L884
.L877:
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 2
	cmp	w0, wzr
	beq	.L879
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	ldr	w0, [x29, 44]
	lsr	w0, w0, 16
	sub	w0, w1, w0
	lsl	w1, w0, 16
	ldr	w0, [x29, 44]
	and	w0, w0, 65535
	add	w1, w1, w0
	ldr	w0, [x29, 44]
	and	w0, w0, 8192
	add	w0, w1, w0
	str	w0, [x29, 44]
.L879:
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 4
	cmp	w0, wzr
	beq	.L880
	ldr	w0, [x29, 44]
	and	w1, w0, -65536
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w2, w0, 6
	ldr	w0, [x29, 44]
	sub	w0, w2, w0
	and	w0, w0, 65535
	add	w1, w1, w0
	ldr	w0, [x29, 44]
	and	w0, w0, 8192
	add	w0, w1, w0
	str	w0, [x29, 44]
.L880:
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 8
	cmp	w0, wzr
	beq	.L881
	ldr	w0, [x29, 44]
	and	w0, w0, 4095
	lsl	w1, w0, 16
	ldr	w0, [x29, 44]
	lsr	w0, w0, 16
	and	w0, w0, 4095
	add	w1, w1, w0
	ldr	w0, [x29, 44]
	and	w0, w0, 8192
	add	w0, w1, w0
	str	w0, [x29, 44]
.L881:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L882
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	add	w2, w0, 1
	adrp	x1, gesture_num
	add	x1, x1, :lo12:gesture_num
	str	w2, [x1]
	sxtw	x0, w0
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldr	w1, [x29, 44]
	str	w1, [x0]
	mov	w0, 1
	b	.L884
.L882:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	sxtw	x0, w0
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x1, x1, x0
	add	x0, x29, 44
	mov	w2, 1
	bl	GestureDistance
	mov	w1, w0
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	ldr	w0, [x0]
	cmp	w1, w0
	bgt	.L883
	mov	w0, 0
	b	.L884
.L883:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	add	w2, w0, 1
	adrp	x1, gesture_num
	add	x1, x1, :lo12:gesture_num
	str	w2, [x1]
	sxtw	x0, w0
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldr	w1, [x29, 44]
	str	w1, [x0]
	mov	w0, 1
.L884:
	ldp	x29, x30, [sp], 48
	ret
	.size	GesturePush, .-GesturePush
	.align	2
	.type	GestureInit, %function
GestureInit:
	sub	sp, sp, #16
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w1, [x0]
	adrp	x0, gesture_num_last
	add	x0, x0, :lo12:gesture_num_last
	str	w1, [x0]
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	str	wzr, [x0]
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	str	wzr, [x0]
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	ldr	w0, [x0]
	cmp	w0, wzr
	blt	.L886
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	ldr	w0, [x0]
	cmp	w0, 64
	ble	.L887
.L886:
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	mov	w1, 2
	str	w1, [x0]
.L887:
	str	wzr, [sp, 12]
	b	.L888
.L889:
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	wzr, [x0, 4]
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [sp, 12]
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	wzr, [x0, 4]
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L888:
	ldr	w0, [sp, 12]
	cmp	w0, 4
	ble	.L889
	add	sp, sp, 16
	ret
	.size	GestureInit, .-GestureInit
	.align	2
	.type	GestureStretch, %function
GestureStretch:
	sub	sp, sp, #32
	str	wzr, [sp, 28]
	mov	w0, 65535
	str	w0, [sp, 24]
	str	wzr, [sp, 20]
	mov	w0, 65535
	str	w0, [sp, 16]
	str	wzr, [sp, 8]
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	cmp	w0, 255
	ble	.L891
	mov	w0, 0
	b	.L892
.L891:
	str	wzr, [sp, 12]
	b	.L893
.L899:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [sp, 28]
	cmp	w1, w0
	bls	.L894
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [sp, 28]
.L894:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [sp, 24]
	cmp	w1, w0
	bcs	.L895
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [sp, 24]
.L895:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [sp, 20]
	cmp	w1, w0
	bls	.L896
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [sp, 20]
.L896:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [sp, 16]
	cmp	w1, w0
	bcs	.L897
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [sp, 16]
.L897:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w0, w0, 8192
	cmp	w0, wzr
	beq	.L898
	ldr	w0, [sp, 8]
	add	w0, w0, 1
	str	w0, [sp, 8]
.L898:
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L893:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [sp, 12]
	cmp	w1, w0
	blt	.L899
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	add	w1, w0, 3
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 2
	mov	w1, w0
	ldr	w0, [sp, 8]
	cmp	w1, w0
	blt	.L900
	ldr	w0, [sp, 8]
	cmp	w0, 4
	ble	.L901
.L900:
	mov	w0, 0
	b	.L892
.L901:
	adrp	x0, gesture_min_x
	add	x0, x0, :lo12:gesture_min_x
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [sp, 24]
	add	w1, w1, w0
	ldr	w0, [sp, 28]
	cmp	w1, w0
	bcs	.L902
	adrp	x0, gesture_min_y
	add	x0, x0, :lo12:gesture_min_y
	ldr	w0, [x0]
	mov	w1, w0
	ldr	w0, [sp, 16]
	add	w1, w1, w0
	ldr	w0, [sp, 20]
	cmp	w1, w0
	bcc	.L903
.L902:
	mov	w0, 0
	b	.L892
.L903:
	adrp	x0, size_ratio
	add	x0, x0, :lo12:size_ratio
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L904
	ldr	w1, [sp, 28]
	ldr	w0, [sp, 24]
	sub	w1, w1, w0
	mov	w0, 100
	mul	w1, w1, w0
	ldr	w2, [sp, 20]
	ldr	w0, [sp, 16]
	sub	w0, w2, w0
	udiv	w1, w1, w0
	adrp	x0, size_ratio
	add	x0, x0, :lo12:size_ratio
	ldr	w0, [x0]
	cmp	w1, w0
	bcc	.L905
	ldr	w1, [sp, 20]
	ldr	w0, [sp, 16]
	sub	w1, w1, w0
	mov	w0, 100
	mul	w1, w1, w0
	ldr	w2, [sp, 28]
	ldr	w0, [sp, 24]
	sub	w0, w2, w0
	udiv	w1, w1, w0
	adrp	x0, size_ratio
	add	x0, x0, :lo12:size_ratio
	ldr	w0, [x0]
	cmp	w1, w0
	bcs	.L904
.L905:
	mov	w0, 0
	b	.L892
.L904:
	str	wzr, [sp, 12]
	b	.L906
.L907:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x2, x1, x0
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [sp, 24]
	sub	w1, w1, w0
	mov	w0, w1
	lsl	w0, w0, 8
	sub	w1, w0, w1
	ldr	w3, [sp, 28]
	ldr	w0, [sp, 24]
	sub	w0, w3, w0
	udiv	w0, w1, w0
	uxth	w0, w0
	strh	w0, [x2, 2]
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x1, x1, x0
	ldrsw	x0, [sp, 12]
	lsl	x2, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldr	w0, [sp, 16]
	sub	w2, w2, w0
	mov	w0, w2
	lsl	w0, w0, 8
	sub	w2, w0, w2
	ldr	w3, [sp, 20]
	ldr	w0, [sp, 16]
	sub	w0, w3, w0
	udiv	w0, w2, w0
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L906:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [sp, 12]
	cmp	w1, w0
	blt	.L907
	mov	w0, 1
.L892:
	add	sp, sp, 32
	ret
	.size	GestureStretch, .-GestureStretch
	.align	2
	.type	GestureLength, %function
GestureLength:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	wzr, [x29, 24]
	mov	w0, 1
	str	w0, [x29, 28]
	b	.L909
.L910:
	ldrsw	x0, [x29, 28]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x3, x1, x0
	ldrsw	x0, [x29, 28]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	mov	w2, 1
	mov	x1, x0
	mov	x0, x3
	bl	GestureDistance
	mov	w1, w0
	ldr	w0, [x29, 24]
	add	w0, w0, w1
	str	w0, [x29, 24]
	ldr	w0, [x29, 28]
	add	w0, w0, 1
	str	w0, [x29, 28]
.L909:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [x29, 28]
	cmp	w1, w0
	blt	.L910
	ldr	w0, [x29, 24]
	ldp	x29, x30, [sp], 32
	ret
	.size	GestureLength, .-GestureLength
	.align	2
	.type	GestureStandard, %function
GestureStandard:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	wzr, [x29, 32]
	str	wzr, [x29, 28]
	bl	GestureLength
	str	w0, [x29, 24]
	adrp	x0, config_static+1024
	add	x0, x0, :lo12:config_static+1024
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	ldr	w1, [x1]
	and	w1, w1, 268435455
	str	w1, [x0]
	adrp	x0, config_static+1148
	add	x0, x0, :lo12:config_static+1148
	adrp	x1, gesture_num
	add	x1, x1, :lo12:gesture_num
	ldr	w1, [x1]
	sxtw	x1, w1
	lsl	x1, x1, 2
	sub	x2, x1, #4
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	add	x1, x2, x1
	ldr	w1, [x1]
	and	w1, w1, 268435455
	str	w1, [x0]
	mov	w0, 1
	str	w0, [x29, 44]
	str	wzr, [x29, 40]
	b	.L913
.L920:
	b	.L914
.L916:
	ldrsw	x0, [x29, 40]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x3, x1, x0
	ldrsw	x0, [x29, 40]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	mov	w2, 1
	mov	x1, x0
	mov	x0, x3
	bl	GestureDistance
	str	w0, [x29, 32]
	ldr	w1, [x29, 28]
	ldr	w0, [x29, 32]
	add	w0, w1, w0
	str	w0, [x29, 28]
	ldr	w1, [x29, 28]
	mov	w0, w1
	lsl	w0, w0, 5
	sub	w1, w0, w1
	ldr	w2, [x29, 24]
	ldr	w0, [x29, 44]
	mul	w0, w2, w0
	cmp	w1, w0
	blt	.L914
	b	.L915
.L914:
	ldr	w0, [x29, 40]
	add	w0, w0, 1
	str	w0, [x29, 40]
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [x29, 40]
	cmp	w1, w0
	blt	.L916
.L915:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [x29, 40]
	cmp	w1, w0
	bge	.L912
	ldr	w0, [x29, 32]
	cmp	w0, wzr
	beq	.L912
	ldrsw	x0, [x29, 44]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	str	wzr, [x0]
	ldrsw	x0, [x29, 40]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w3, w0
	ldrsw	x0, [x29, 40]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldrsw	x0, [x29, 40]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	sub	w2, w2, w0
	ldr	w1, [x29, 24]
	ldr	w0, [x29, 44]
	mul	w0, w1, w0
	mov	w1, 2115
	movk	w1, 0x8421, lsl 16
	smull	x1, w0, w1
	lsr	x1, x1, 32
	add	w1, w0, w1
	asr	w1, w1, 4
	asr	w0, w0, 31
	sub	w1, w1, w0
	ldr	w0, [x29, 28]
	sub	w1, w1, w0
	ldr	w0, [x29, 32]
	add	w0, w1, w0
	mul	w1, w2, w0
	ldr	w0, [x29, 32]
	sdiv	w0, w1, w0
	add	w0, w3, w0
	str	w0, [x29, 36]
	ldr	w0, [x29, 36]
	cmp	w0, wzr
	bge	.L918
	str	wzr, [x29, 36]
.L918:
	ldrsw	x0, [x29, 44]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldr	w1, [x29, 36]
	uxth	w1, w1
	strh	w1, [x0, 2]
	ldrsw	x0, [x29, 40]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w3, w0
	ldrsw	x0, [x29, 40]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldrsw	x0, [x29, 40]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w2, w2, w0
	ldr	w1, [x29, 24]
	ldr	w0, [x29, 44]
	mul	w0, w1, w0
	mov	w1, 2115
	movk	w1, 0x8421, lsl 16
	smull	x1, w0, w1
	lsr	x1, x1, 32
	add	w1, w0, w1
	asr	w1, w1, 4
	asr	w0, w0, 31
	sub	w1, w1, w0
	ldr	w0, [x29, 28]
	sub	w1, w1, w0
	ldr	w0, [x29, 32]
	add	w0, w1, w0
	mul	w1, w2, w0
	ldr	w0, [x29, 32]
	sdiv	w0, w1, w0
	add	w0, w3, w0
	str	w0, [x29, 36]
	ldr	w0, [x29, 36]
	cmp	w0, wzr
	bge	.L919
	str	wzr, [x29, 36]
.L919:
	ldrsw	x0, [x29, 44]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x1, x1, x0
	ldr	w0, [x29, 36]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	ldr	w0, [x29, 40]
	sub	w0, w0, #1
	str	w0, [x29, 40]
	ldr	w1, [x29, 28]
	ldr	w0, [x29, 32]
	sub	w0, w1, w0
	str	w0, [x29, 28]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 44]
.L913:
	ldr	w0, [x29, 44]
	cmp	w0, 30
	ble	.L920
.L912:
	ldp	x29, x30, [sp], 48
	ret
	.size	GestureStandard, .-GestureStandard
	.align	2
	.type	GestureModel, %function
GestureModel:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	str	x0, [x29, 40]
	str	w1, [x29, 36]
	str	w2, [x29, 32]
	str	x3, [x29, 24]
	add	x0, x29, 64
	mov	w1, -2
	str	w1, [x0]
	add	x0, x29, 64
	mov	w1, -1
	str	w1, [x0, 4]
	add	x0, x29, 64
	str	wzr, [x0, 8]
	add	x0, x29, 64
	mov	w1, 1
	str	w1, [x0, 12]
	add	x0, x29, 64
	mov	w1, 2
	str	w1, [x0, 16]
	ldr	x0, [x29, 40]
	cmp	x0, xzr
	beq	.L922
	ldr	w0, [x29, 32]
	cmp	w0, wzr
	ble	.L922
	ldr	w0, [x29, 36]
	cmp	w0, wzr
	bgt	.L923
.L922:
	ldr	x0, [x29, 24]
	mov	w1, 2147483647
	str	w1, [x0]
	mov	w0, 2147483647
	b	.L940
.L923:
	mov	w0, 2147483647
	str	w0, [x29, 88]
	str	wzr, [x29, 84]
	str	wzr, [x29, 100]
	b	.L925
.L937:
	str	wzr, [x29, 96]
	b	.L926
.L936:
	str	wzr, [x29, 92]
	str	wzr, [x29, 108]
	str	wzr, [x29, 104]
	b	.L927
.L933:
	add	x0, x29, 64
	ldrsw	x1, [x29, 96]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 104]
	add	w0, w1, w0
	cmp	w0, wzr
	blt	.L928
	add	x0, x29, 64
	ldrsw	x1, [x29, 96]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 104]
	add	w0, w1, w0
	cmp	w0, 31
	ble	.L929
.L928:
	b	.L930
.L929:
	ldr	w0, [x29, 104]
	and	w0, w0, 1
	cmp	w0, wzr
	bne	.L931
	ldrsw	x1, [x29, 100]
	mov	x0, x1
	lsl	x0, x0, 3
	add	x0, x0, x1
	lsl	x0, x0, 3
	mov	x1, x0
	ldr	x0, [x29, 40]
	add	x1, x0, x1
	ldr	w0, [x29, 104]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	sxtw	x0, w0
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0, 8]
	and	w0, w0, 16711935
	str	w0, [x29, 56]
	b	.L932
.L931:
	ldrsw	x1, [x29, 100]
	mov	x0, x1
	lsl	x0, x0, 3
	add	x0, x0, x1
	lsl	x0, x0, 3
	mov	x1, x0
	ldr	x0, [x29, 40]
	add	x1, x0, x1
	ldr	w0, [x29, 104]
	lsr	w2, w0, 31
	add	w0, w2, w0
	asr	w0, w0, 1
	sxtw	x0, w0
	lsl	x0, x0, 2
	add	x0, x1, x0
	ldr	w0, [x0, 8]
	lsr	w0, w0, 8
	and	w0, w0, 16711935
	str	w0, [x29, 56]
.L932:
	add	x0, x29, 64
	ldrsw	x1, [x29, 96]
	ldr	w1, [x0, x1, lsl 2]
	ldr	w0, [x29, 104]
	add	w0, w1, w0
	sxtw	x0, w0
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	add	x1, x29, 56
	mov	w2, 0
	bl	GestureDistance
	mov	w1, w0
	ldr	w0, [x29, 108]
	add	w0, w0, w1
	str	w0, [x29, 108]
	ldr	w0, [x29, 92]
	add	w0, w0, 1
	str	w0, [x29, 92]
.L930:
	ldr	w0, [x29, 104]
	add	w0, w0, 1
	str	w0, [x29, 104]
.L927:
	ldr	w0, [x29, 104]
	cmp	w0, 31
	ble	.L933
	ldr	w0, [x29, 92]
	cmp	w0, wzr
	bne	.L934
	b	.L935
.L934:
	ldr	w1, [x29, 108]
	ldr	w0, [x29, 92]
	sdiv	w2, w1, w0
	ldrsw	x1, [x29, 100]
	mov	x0, x1
	lsl	x0, x0, 3
	add	x0, x0, x1
	lsl	x0, x0, 3
	mov	x1, x0
	ldr	x0, [x29, 40]
	add	x0, x0, x1
	ldr	w0, [x0]
	mul	w0, w2, w0
	add	w1, w0, 15
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 4
	str	w0, [x29, 108]
	ldr	w1, [x29, 108]
	ldr	w0, [x29, 88]
	cmp	w1, w0
	bge	.L935
	ldr	w0, [x29, 100]
	str	w0, [x29, 84]
	ldr	w0, [x29, 108]
	str	w0, [x29, 88]
.L935:
	ldr	w0, [x29, 96]
	add	w0, w0, 1
	str	w0, [x29, 96]
.L926:
	ldr	w0, [x29, 96]
	cmp	w0, 4
	bls	.L936
	ldr	w0, [x29, 100]
	add	w0, w0, 1
	str	w0, [x29, 100]
.L925:
	ldr	w1, [x29, 100]
	ldr	w0, [x29, 36]
	cmp	w1, w0
	blt	.L937
	ldr	w1, [x29, 88]
	ldr	w0, [x29, 32]
	cmp	w1, w0
	bge	.L938
	ldrsw	x1, [x29, 84]
	mov	x0, x1
	lsl	x0, x0, 3
	add	x0, x0, x1
	lsl	x0, x0, 3
	mov	x1, x0
	ldr	x0, [x29, 40]
	add	x0, x0, x1
	ldr	w1, [x0, 4]
	ldr	x0, [x29, 24]
	str	w1, [x0]
	b	.L939
.L938:
	ldr	x0, [x29, 24]
	mov	w1, 2147483647
	str	w1, [x0]
.L939:
	ldr	w0, [x29, 88]
.L940:
	ldp	x29, x30, [sp], 112
	ret
	.size	GestureModel, .-GestureModel
	.align	2
	.type	ChangeXY, %function
ChangeXY:
	sub	sp, sp, #16
	str	wzr, [sp, 12]
	b	.L942
.L944:
	ldrsw	x0, [sp, 12]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrsw	x1, [sp, 12]
	lsl	x2, x1, 2
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	add	x1, x2, x1
	ldr	w1, [x1]
	and	w1, w1, 4095
	lsl	w2, w1, 16
	ldrsw	x1, [sp, 12]
	lsl	x3, x1, 2
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	add	x1, x3, x1
	ldr	w1, [x1]
	lsr	w1, w1, 16
	add	w2, w2, w1
	ldrsw	x1, [sp, 12]
	lsl	x3, x1, 2
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	add	x1, x3, x1
	ldr	w1, [x1]
	and	w1, w1, 8192
	add	w1, w2, w1
	str	w1, [x0]
	ldr	w0, [sp, 12]
	add	w0, w0, 1
	str	w0, [sp, 12]
.L942:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [sp, 12]
	cmp	w1, w0
	bge	.L941
	ldr	w0, [sp, 12]
	cmp	w0, 255
	ble	.L944
.L941:
	add	sp, sp, 16
	ret
	.size	ChangeXY, .-ChangeXY
	.align	2
	.type	GestureSet, %function
GestureSet:
	sub	sp, sp, #16
	str	x0, [sp, 8]
	ldr	x0, [sp, 8]
	cmp	x0, xzr
	bne	.L946
	b	.L945
.L946:
	ldr	x0, [sp, 8]
	ldr	w0, [x0]
	cmp	w0, 64
	bhi	.L948
	ldr	x0, [sp, 8]
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	str	w1, [x0]
	b	.L949
.L948:
	adrp	x0, gesture_dis_min
	add	x0, x0, :lo12:gesture_dis_min
	str	wzr, [x0]
.L949:
	ldr	x0, [sp, 8]
	add	x0, x0, 4
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L950
	ldr	x0, [sp, 8]
	add	x0, x0, 4
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, gesture_threshold
	add	x0, x0, :lo12:gesture_threshold
	str	w1, [x0]
	b	.L951
.L950:
	adrp	x0, gesture_threshold
	add	x0, x0, :lo12:gesture_threshold
	mov	w1, 4095
	str	w1, [x0]
.L951:
	ldr	x0, [sp, 8]
	add	x0, x0, 8
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, gesture_threshold
	add	x0, x0, :lo12:gesture_threshold
	str	w1, [x0, 4]
	ldr	x0, [sp, 8]
	add	x0, x0, 12
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L952
	ldr	x0, [sp, 8]
	add	x0, x0, 12
	ldr	w0, [x0]
	b	.L953
.L952:
	mov	w0, 4
.L953:
	adrp	x1, x_scale
	add	x1, x1, :lo12:x_scale
	str	w0, [x1]
	ldr	x0, [sp, 8]
	add	x0, x0, 16
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L954
	ldr	x0, [sp, 8]
	add	x0, x0, 16
	ldr	w0, [x0]
	b	.L955
.L954:
	mov	w0, 4
.L955:
	adrp	x1, y_scale
	add	x1, x1, :lo12:y_scale
	str	w0, [x1]
	ldr	x0, [sp, 8]
	add	x0, x0, 20
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L956
	adrp	x0, double_down
	add	x0, x0, :lo12:double_down
	mov	w1, 2
	str	w1, [x0]
	adrp	x0, double_up
	add	x0, x0, :lo12:double_up
	mov	w1, 30
	str	w1, [x0]
	b	.L957
.L956:
	ldr	x0, [sp, 8]
	add	x0, x0, 20
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, double_down
	add	x0, x0, :lo12:double_down
	str	w1, [x0]
	ldr	x0, [sp, 8]
	add	x0, x0, 20
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, double_up
	add	x0, x0, :lo12:double_up
	str	w1, [x0]
.L957:
	ldr	x0, [sp, 8]
	ldr	w1, [x0, 24]
	adrp	x0, multi_set
	add	x0, x0, :lo12:multi_set
	str	w1, [x0]
	ldr	x0, [sp, 8]
	ldr	w1, [x0, 28]
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	str	w1, [x0]
	ldr	x0, [sp, 8]
	add	x0, x0, 36
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L958
	ldr	x0, [sp, 8]
	add	x0, x0, 36
	ldr	w0, [x0]
	and	w1, w0, 65535
	adrp	x0, gesture_min_x
	add	x0, x0, :lo12:gesture_min_x
	str	w1, [x0]
	ldr	x0, [sp, 8]
	add	x0, x0, 36
	ldr	w0, [x0]
	lsr	w0, w0, 16
	mov	w1, w0
	adrp	x0, gesture_min_y
	add	x0, x0, :lo12:gesture_min_y
	str	w1, [x0]
	b	.L959
.L958:
	adrp	x0, gesture_min_x
	add	x0, x0, :lo12:gesture_min_x
	mov	w1, 128
	str	w1, [x0]
	adrp	x0, gesture_min_y
	add	x0, x0, :lo12:gesture_min_y
	mov	w1, 192
	str	w1, [x0]
.L959:
	ldr	x0, [sp, 8]
	ldr	w1, [x0, 40]
	adrp	x0, size_ratio
	add	x0, x0, :lo12:size_ratio
	str	w1, [x0]
.L945:
	add	sp, sp, 16
	ret
	.size	GestureSet, .-GestureSet
	.align	2
	.type	GestureOrientation, %function
GestureOrientation:
	sub	sp, sp, #32
	str	w0, [sp, 12]
	ldr	w0, [sp, 12]
	cmp	w0, wzr
	bne	.L961
	b	.L960
.L961:
	ldr	w0, [sp, 12]
	cmp	w0, 2
	bne	.L963
	str	wzr, [sp, 28]
	b	.L964
.L965:
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	mov	w0, 255
	sub	w0, w0, w1
	str	w0, [sp, 24]
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrsw	x1, [sp, 28]
	add	x1, x1, 256
	lsl	x2, x1, 2
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	add	x1, x2, x1
	ldrh	w1, [x1]
	ubfx	x1, x1, 0, 12
	uxth	w1, w1
	strh	w1, [x0, 2]
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x1, x1, x0
	ldr	w0, [sp, 24]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L964:
	ldr	w0, [sp, 28]
	cmp	w0, 31
	ble	.L965
	b	.L960
.L963:
	ldr	w0, [sp, 12]
	cmp	w0, 1
	beq	.L966
	ldr	w0, [sp, 12]
	cmp	w0, 3
	bne	.L967
.L966:
	str	wzr, [sp, 28]
	b	.L968
.L969:
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrsw	x1, [sp, 28]
	add	x1, x1, 256
	lsl	x2, x1, 2
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	add	x1, x2, x1
	ldrh	w1, [x1, 2]
	mov	w2, 255
	sub	w1, w2, w1
	uxth	w1, w1
	strh	w1, [x0, 2]
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x1, x1, x0
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x2, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x2, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	mov	w0, 255
	sub	w0, w0, w2
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L968:
	ldr	w0, [sp, 28]
	cmp	w0, 31
	ble	.L969
	b	.L960
.L967:
	ldr	w0, [sp, 12]
	cmp	w0, 4
	bne	.L960
	str	wzr, [sp, 28]
	b	.L970
.L971:
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	mov	w0, 255
	sub	w0, w0, w1
	str	w0, [sp, 24]
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x1, x1, x0
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x2, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x2, x0
	ldrh	w0, [x0, 2]
	and	w0, w0, 4095
	uxth	w2, w0
	ldrh	w0, [x1]
	bfi	w0, w2, 0, 12
	strh	w0, [x1]
	ldrsw	x0, [sp, 28]
	add	x0, x0, 256
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldr	w1, [sp, 24]
	uxth	w1, w1
	strh	w1, [x0, 2]
	ldr	w0, [sp, 28]
	add	w0, w0, 1
	str	w0, [sp, 28]
.L970:
	ldr	w0, [sp, 28]
	cmp	w0, 31
	ble	.L971
.L960:
	add	sp, sp, 32
	ret
	.size	GestureOrientation, .-GestureOrientation
	.align	2
	.type	GestureDeal, %function
GestureDeal:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L973
	b	.L974
.L973:
	adrp	x0, gesture_multi
	add	x0, x0, :lo12:gesture_multi
	ldr	w1, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L975
	b	.L974
.L975:
	adrp	x0, double_click
	add	x0, x0, :lo12:double_click
	ldr	w0, [x0]
	mov	w1, w0
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L976
	b	.L974
.L976:
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w0, w0, 1
	cmp	w0, wzr
	beq	.L977
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w1, w0, -2
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	str	w1, [x0]
	bl	ChangeXY
.L977:
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w0, w0, 2
	cmp	w0, wzr
	bne	.L978
	mov	w0, 0
	b	.L999
.L978:
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w1, w0, -3
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	str	w1, [x0]
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 256
	cmp	w0, wzr
	bne	.L980
	bl	GestureLRUD
	mov	w1, w0
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L980
	b	.L974
.L980:
	bl	GestureStretch
	cmp	w0, wzr
	bne	.L981
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 256
	cmp	w0, wzr
	beq	.L982
	bl	GestureLRUD
	mov	w1, w0
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L982
	b	.L974
.L982:
	b	.L974
.L981:
	bl	GestureStandard
	str	wzr, [x29, 72]
	b	.L983
.L990:
	add	x0, x29, 24
	ldrsw	x1, [x29, 72]
	mov	w2, 2147483647
	str	w2, [x0, x1, lsl 2]
	ldr	w0, [x29, 72]
	bl	GestureOrientation
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 240
	cmp	w0, wzr
	bne	.L984
	ldr	w0, [x29, 72]
	cmp	w0, wzr
	beq	.L985
.L984:
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 240
	cmp	w0, wzr
	beq	.L986
	ldr	w0, [x29, 72]
	mov	w1, 16
	lsl	w0, w1, w0
	mov	w1, w0
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w1, w0
	cmp	w0, wzr
	beq	.L986
.L985:
	adrp	x0, gesture_threshold
	add	x0, x0, :lo12:gesture_threshold
	ldr	w1, [x0]
	add	x2, x29, 64
	adrp	x0, model_default
	add	x0, x0, :lo12:model_default
	mov	x3, x2
	mov	w2, w1
	mov	w1, 79
	bl	GestureModel
	str	w0, [x29, 56]
	adrp	x0, model_extern
	add	x0, x0, :lo12:model_extern
	ldr	x4, [x0]
	adrp	x0, model_extern_len
	add	x0, x0, :lo12:model_extern_len
	ldr	w1, [x0]
	adrp	x0, gesture_threshold
	add	x0, x0, :lo12:gesture_threshold
	ldr	w2, [x0, 4]
	add	x0, x29, 64
	add	x0, x0, 4
	mov	x3, x0
	mov	x0, x4
	bl	GestureModel
	str	w0, [x29, 60]
	str	wzr, [x29, 76]
	b	.L987
.L989:
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x1, x29, 80
	add	x0, x1, x0
	ldr	w1, [x0, -24]
	add	x0, x29, 24
	ldrsw	x2, [x29, 72]
	ldr	w0, [x0, x2, lsl 2]
	cmp	w1, w0
	bgt	.L988
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x1, x29, 80
	add	x0, x1, x0
	ldr	w2, [x0, -24]
	add	x0, x29, 24
	ldrsw	x1, [x29, 72]
	str	w2, [x0, x1, lsl 2]
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	add	x1, x29, 80
	add	x0, x1, x0
	ldr	w2, [x0, -16]
	add	x0, x29, 40
	ldrsw	x1, [x29, 72]
	str	w2, [x0, x1, lsl 2]
.L988:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L987:
	ldr	w0, [x29, 76]
	cmp	w0, 1
	ble	.L989
.L986:
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L983:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L990
	adrp	x0, gesture_value
	add	x0, x0, :lo12:gesture_value
	mov	w1, 2147483647
	str	w1, [x0]
	str	wzr, [x29, 72]
	b	.L991
.L993:
	add	x0, x29, 40
	ldrsw	x1, [x29, 72]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 64
	ble	.L992
	add	x0, x29, 40
	ldrsw	x1, [x29, 72]
	ldr	w0, [x0, x1, lsl 2]
	cmp	w0, 90
	bgt	.L992
	add	x0, x29, 24
	ldrsw	x1, [x29, 72]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, gesture_value
	add	x0, x0, :lo12:gesture_value
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L992
	add	x0, x29, 24
	ldrsw	x1, [x29, 72]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, gesture_value
	add	x0, x0, :lo12:gesture_value
	str	w1, [x0]
	add	x0, x29, 40
	ldrsw	x1, [x29, 72]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
.L992:
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L991:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L993
	adrp	x0, gesture_value
	add	x0, x0, :lo12:gesture_value
	ldr	w1, [x0]
	mov	w0, 2147483647
	cmp	w1, w0
	beq	.L994
	b	.L974
.L994:
	adrp	x0, gesture_flag
	add	x0, x0, :lo12:gesture_flag
	ldr	w0, [x0]
	and	w0, w0, 256
	cmp	w0, wzr
	beq	.L995
	bl	GestureLRUD
	mov	w1, w0
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	cmp	w0, wzr
	beq	.L995
	b	.L974
.L995:
	str	wzr, [x29, 72]
	b	.L996
.L998:
	add	x0, x29, 24
	ldrsw	x1, [x29, 72]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, gesture_value
	add	x0, x0, :lo12:gesture_value
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L997
	add	x0, x29, 24
	ldrsw	x1, [x29, 72]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, gesture_value
	add	x0, x0, :lo12:gesture_value
	str	w1, [x0]
	add	x0, x29, 40
	ldrsw	x1, [x29, 72]
	ldr	w1, [x0, x1, lsl 2]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	w1, [x0]
.L997:
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L996:
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L998
	nop
.L974:
	bl	GestureInit
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	ldr	w0, [x0]
	str	w0, [x29, 76]
	adrp	x0, gesture_last
	add	x0, x0, :lo12:gesture_last
	str	wzr, [x0]
	ldr	w0, [x29, 76]
.L999:
	ldp	x29, x30, [sp], 80
	ret
	.size	GestureDeal, .-GestureDeal
	.align	2
	.global	gsl_GestureExtern
	.type	gsl_GestureExtern, %function
gsl_GestureExtern:
	sub	sp, sp, #16
	str	x0, [sp, 8]
	str	w1, [sp, 4]
	adrp	x0, model_extern
	add	x0, x0, :lo12:model_extern
	ldr	x1, [sp, 8]
	str	x1, [x0]
	adrp	x0, model_extern_len
	add	x0, x0, :lo12:model_extern_len
	ldr	w1, [sp, 4]
	str	w1, [x0]
	add	sp, sp, 16
	ret
	.size	gsl_GestureExtern, .-gsl_GestureExtern
	.align	2
	.global	gsl_GestureExternInt
	.type	gsl_GestureExternInt, %function
gsl_GestureExternInt:
	sub	sp, sp, #16
	str	x0, [sp, 8]
	str	w1, [sp, 4]
	adrp	x0, model_extern
	add	x0, x0, :lo12:model_extern
	ldr	x1, [sp, 8]
	str	x1, [x0]
	adrp	x0, model_extern_len
	add	x0, x0, :lo12:model_extern_len
	ldr	w1, [sp, 4]
	str	w1, [x0]
	add	sp, sp, 16
	ret
	.size	gsl_GestureExternInt, .-gsl_GestureExternInt
	.align	2
	.type	GestureLRUD, %function
GestureLRUD:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	str	wzr, [x29, 76]
	str	wzr, [x29, 72]
	mov	w0, 5
	str	w0, [x29, 48]
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	add	w1, w0, 7
	cmp	w0, wzr
	csel	w0, w1, w0, lt
	asr	w0, w0, 3
	str	w0, [x29, 44]
	ldr	w1, [x29, 44]
	mov	w0, w1
	lsl	w1, w1, 1
	sub	w0, w0, w1
	lsl	w0, w0, 1
	mov	w1, w0
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	add	w0, w1, w0
	sub	w0, w0, #1
	str	w0, [x29, 40]
	str	wzr, [x29, 52]
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w0, w0, 1
	cmp	w0, wzr
	beq	.L1003
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w1, w0, -2
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	str	w1, [x0]
	bl	ChangeXY
.L1003:
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w0, w0, 4
	cmp	w0, wzr
	bne	.L1004
	mov	w0, 0
	b	.L1005
.L1004:
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	ldr	w0, [x0]
	and	w1, w0, -5
	adrp	x0, gesture_deal
	add	x0, x0, :lo12:gesture_deal
	str	w1, [x0]
	ldrsw	x0, [x29, 44]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldrsw	x0, [x29, 40]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	add	w0, w2, w0
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [x29, 36]
	ldrsw	x0, [x29, 44]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldrsw	x0, [x29, 40]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	add	w0, w2, w0
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	str	w0, [x29, 32]
	adrp	x0, sen_num_nokey
	add	x0, x0, :lo12:sen_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w0, [x0]
	cmp	w0, 100
	ble	.L1006
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w0, [x0]
	mov	w2, 100
	sdiv	w3, w0, w2
	mov	w2, 100
	mul	w2, w3, w2
	sub	w0, w0, w2
	b	.L1007
.L1006:
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w0, [x0]
.L1007:
	sdiv	w0, w1, w0
	str	w0, [x29, 28]
	adrp	x0, drv_num_nokey
	add	x0, x0, :lo12:drv_num_nokey
	ldr	w0, [x0]
	lsl	w1, w0, 6
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w0, [x0]
	cmp	w0, 100
	ble	.L1008
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w0, [x0]
	mov	w2, 100
	sdiv	w3, w0, w2
	mov	w2, 100
	mul	w2, w3, w2
	sub	w0, w0, w2
	b	.L1009
.L1008:
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w0, [x0]
.L1009:
	sdiv	w0, w1, w0
	str	w0, [x29, 24]
	ldrsw	x0, [x29, 44]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [x29, 64]
	ldrsw	x0, [x29, 44]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [x29, 68]
	ldrsw	x0, [x29, 44]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [x29, 56]
	ldrsw	x0, [x29, 44]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [x29, 60]
	ldr	w0, [x29, 44]
	add	w0, w0, 1
	str	w0, [x29, 76]
	b	.L1010
.L1023:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 36]
	sub	w0, w1, w0
	sxtw	x0, w0
	eor	x1, x0, x0, asr 63
	sub	x0, x1, x0, asr 63
	mov	w1, w0
	ldr	w0, [x29, 28]
	cmp	w1, w0
	bge	.L1011
	ldr	w0, [x29, 72]
	orr	w0, w0, 1
	str	w0, [x29, 72]
	b	.L1012
.L1011:
	ldr	w0, [x29, 72]
	orr	w0, w0, 2
	str	w0, [x29, 72]
.L1012:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 32]
	sub	w0, w1, w0
	sxtw	x0, w0
	eor	x1, x0, x0, asr 63
	sub	x0, x1, x0, asr 63
	mov	w1, w0
	ldr	w0, [x29, 24]
	cmp	w1, w0
	bge	.L1013
	ldr	w0, [x29, 72]
	orr	w0, w0, 16
	str	w0, [x29, 72]
	b	.L1014
.L1013:
	ldr	w0, [x29, 72]
	orr	w0, w0, 32
	str	w0, [x29, 72]
.L1014:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	sub	w1, w2, w0
	ldr	w0, [x29, 48]
	cmp	w1, w0
	ble	.L1015
	ldr	w0, [x29, 72]
	orr	w0, w0, 256
	str	w0, [x29, 72]
	b	.L1016
.L1015:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w2, w0
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	sub	w1, w2, w0
	ldr	w0, [x29, 48]
	neg	w0, w0
	cmp	w1, w0
	bge	.L1016
	ldr	w0, [x29, 72]
	orr	w0, w0, 512
	str	w0, [x29, 72]
.L1016:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w1, w2, w0
	ldr	w0, [x29, 48]
	cmp	w1, w0
	ble	.L1017
	ldr	w0, [x29, 72]
	orr	w0, w0, 4096
	str	w0, [x29, 72]
	b	.L1018
.L1017:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w2, w0
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 2
	sub	x1, x0, #4
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	sub	w1, w2, w0
	ldr	w0, [x29, 48]
	neg	w0, w0
	cmp	w1, w0
	bge	.L1018
	ldr	w0, [x29, 72]
	orr	w0, w0, 8192
	str	w0, [x29, 72]
.L1018:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 64]
	cmp	w1, w0
	bge	.L1019
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [x29, 64]
.L1019:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 68]
	cmp	w1, w0
	ble	.L1020
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	str	w0, [x29, 68]
.L1020:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 56]
	cmp	w1, w0
	bge	.L1021
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [x29, 56]
.L1021:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 60]
	cmp	w1, w0
	ble	.L1022
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	str	w0, [x29, 60]
.L1022:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L1010:
	ldr	w1, [x29, 76]
	ldr	w0, [x29, 40]
	cmp	w1, w0
	ble	.L1023
	ldr	w1, [x29, 72]
	mov	w0, 8241
	cmp	w1, w0
	beq	.L1024
	ldr	w1, [x29, 72]
	mov	w0, 8497
	cmp	w1, w0
	beq	.L1024
	ldr	w1, [x29, 72]
	mov	w0, 8753
	cmp	w1, w0
	beq	.L1024
	ldr	w1, [x29, 72]
	mov	w0, 9009
	cmp	w1, w0
	bne	.L1025
.L1024:
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w0, [x0]
	add	w0, w0, 99
	cmp	w0, 198
	bls	.L1026
	ldr	w1, [x29, 60]
	ldr	w0, [x29, 56]
	sub	w2, w1, w0
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w1, [x0]
	mov	w0, 34079
	movk	w0, 0x51eb, lsl 16
	smull	x0, w1, w0
	lsr	x0, x0, 32
	asr	w3, w0, 5
	asr	w0, w1, 31
	sub	w0, w3, w0
	lsl	w0, w0, 6
	cmp	w2, w0
	bge	.L1027
.L1026:
	mov	w0, 41468
	str	w0, [x29, 52]
	b	.L1028
.L1027:
	b	.L1028
.L1025:
	ldr	w1, [x29, 72]
	mov	w0, 4145
	cmp	w1, w0
	beq	.L1029
	ldr	w1, [x29, 72]
	mov	w0, 4401
	cmp	w1, w0
	beq	.L1029
	ldr	w1, [x29, 72]
	mov	w0, 4657
	cmp	w1, w0
	beq	.L1029
	ldr	w1, [x29, 72]
	mov	w0, 4913
	cmp	w1, w0
	bne	.L1030
.L1029:
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w0, [x0]
	add	w0, w0, 99
	cmp	w0, 198
	bls	.L1031
	ldr	w1, [x29, 60]
	ldr	w0, [x29, 56]
	sub	w2, w1, w0
	adrp	x0, y_scale
	add	x0, x0, :lo12:y_scale
	ldr	w1, [x0]
	mov	w0, 34079
	movk	w0, 0x51eb, lsl 16
	smull	x0, w1, w0
	lsr	x0, x0, 32
	asr	w3, w0, 5
	asr	w0, w1, 31
	sub	w0, w3, w0
	lsl	w0, w0, 6
	cmp	w2, w0
	bge	.L1032
.L1031:
	mov	w0, 41469
	str	w0, [x29, 52]
	b	.L1028
.L1032:
	b	.L1028
.L1030:
	ldr	w0, [x29, 72]
	cmp	w0, 531
	beq	.L1033
	ldr	w1, [x29, 72]
	mov	w0, 4627
	cmp	w1, w0
	beq	.L1033
	ldr	w1, [x29, 72]
	mov	w0, 8723
	cmp	w1, w0
	beq	.L1033
	ldr	w1, [x29, 72]
	mov	w0, 8723
	cmp	w1, w0
	bne	.L1034
.L1033:
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w0, [x0]
	add	w0, w0, 99
	cmp	w0, 198
	bls	.L1035
	ldr	w1, [x29, 68]
	ldr	w0, [x29, 64]
	sub	w2, w1, w0
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w1, [x0]
	mov	w0, 34079
	movk	w0, 0x51eb, lsl 16
	smull	x0, w1, w0
	lsr	x0, x0, 32
	asr	w3, w0, 5
	asr	w0, w1, 31
	sub	w0, w3, w0
	lsl	w0, w0, 6
	cmp	w2, w0
	bge	.L1036
.L1035:
	mov	w0, 41467
	str	w0, [x29, 52]
	b	.L1028
.L1036:
	b	.L1028
.L1034:
	ldr	w0, [x29, 72]
	cmp	w0, 275
	beq	.L1037
	ldr	w1, [x29, 72]
	mov	w0, 4371
	cmp	w1, w0
	beq	.L1037
	ldr	w1, [x29, 72]
	mov	w0, 8467
	cmp	w1, w0
	beq	.L1037
	ldr	w1, [x29, 72]
	mov	w0, 12563
	cmp	w1, w0
	bne	.L1028
.L1037:
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w0, [x0]
	add	w0, w0, 99
	cmp	w0, 198
	bls	.L1038
	ldr	w1, [x29, 68]
	ldr	w0, [x29, 64]
	sub	w2, w1, w0
	adrp	x0, x_scale
	add	x0, x0, :lo12:x_scale
	ldr	w1, [x0]
	mov	w0, 34079
	movk	w0, 0x51eb, lsl 16
	smull	x0, w1, w0
	lsr	x0, x0, 32
	asr	w3, w0, 5
	asr	w0, w1, 31
	sub	w0, w3, w0
	lsl	w0, w0, 6
	cmp	w2, w0
	bge	.L1028
.L1038:
	mov	w0, 41466
	str	w0, [x29, 52]
.L1028:
	ldr	w0, [x29, 52]
	cmp	w0, wzr
	ble	.L1039
	str	wzr, [x29, 72]
	str	wzr, [x29, 76]
	b	.L1040
.L1042:
	ldrsw	x0, [x29, 76]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	ldr	w0, [x0]
	and	w0, w0, 8192
	cmp	w0, wzr
	beq	.L1041
	ldr	w0, [x29, 72]
	add	w0, w0, 1
	str	w0, [x29, 72]
.L1041:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
.L1040:
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	ldr	w1, [x29, 76]
	cmp	w1, w0
	blt	.L1042
	adrp	x0, gesture_num
	add	x0, x0, :lo12:gesture_num
	ldr	w0, [x0]
	lsr	w1, w0, 31
	add	w0, w1, w0
	asr	w0, w0, 1
	mov	w1, w0
	ldr	w0, [x29, 72]
	cmp	w1, w0
	bgt	.L1039
	ldr	w0, [x29, 72]
	cmp	w0, 3
	ble	.L1039
	ldr	w0, [x29, 52]
	sub	w0, w0, #4
	str	w0, [x29, 52]
.L1039:
	ldr	w0, [x29, 52]
.L1005:
	ldp	x29, x30, [sp], 80
	ret
	.size	GestureLRUD, .-GestureLRUD
	.align	2
	.type	GestureMulti, %function
GestureMulti:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	str	x0, [x29, 24]
	ldr	x0, [x29, 24]
	str	x0, [x29, 40]
	str	wzr, [x29, 76]
	str	wzr, [x29, 72]
	str	wzr, [x29, 68]
	str	wzr, [x29, 64]
	adrp	x0, multi_set
	add	x0, x0, :lo12:multi_set
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L1044
	mov	w0, 5
	str	w0, [x29, 56]
	mov	w0, 8
	str	w0, [x29, 52]
	mov	w0, 16
	str	w0, [x29, 48]
	b	.L1045
.L1044:
	adrp	x0, multi_set
	add	x0, x0, :lo12:multi_set
	ldr	w0, [x0]
	and	w0, w0, 255
	str	w0, [x29, 56]
	adrp	x0, multi_set
	add	x0, x0, :lo12:multi_set
	ldr	w0, [x0]
	lsr	w0, w0, 8
	and	w0, w0, 255
	str	w0, [x29, 52]
	adrp	x0, multi_set
	add	x0, x0, :lo12:multi_set
	ldr	w0, [x0]
	lsr	w0, w0, 16
	and	w0, w0, 255
	str	w0, [x29, 48]
.L1045:
	str	wzr, [x29, 60]
	b	.L1046
.L1049:
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	ldr	x1, [x29, 40]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L1047
	b	.L1048
.L1047:
	ldr	w0, [x29, 76]
	add	w0, w0, 1
	str	w0, [x29, 76]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	ldr	x1, [x29, 40]
	add	x0, x1, x0
	ldrh	w0, [x0, 2]
	mov	w1, w0
	ldr	w0, [x29, 72]
	add	w0, w0, w1
	str	w0, [x29, 72]
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	ldr	x1, [x29, 40]
	add	x0, x1, x0
	ldrh	w0, [x0]
	ubfx	x0, x0, 0, 12
	uxth	w0, w0
	mov	w1, w0
	ldr	w0, [x29, 68]
	add	w0, w0, w1
	str	w0, [x29, 68]
.L1048:
	ldr	w0, [x29, 60]
	add	w0, w0, 1
	str	w0, [x29, 60]
.L1046:
	ldr	w0, [x29, 60]
	cmp	w0, 9
	ble	.L1049
	ldr	w0, [x29, 76]
	cmp	w0, wzr
	bne	.L1050
	str	wzr, [x29, 64]
	adrp	x0, multi_x
	add	x0, x0, :lo12:multi_x
	ldr	w0, [x0, 36]
	cmp	w0, wzr
	blt	.L1051
	adrp	x0, multi_x
	add	x0, x0, :lo12:multi_x
	ldr	w0, [x0, 28]
	cmp	w0, wzr
	blt	.L1051
	adrp	x0, multi_x
	add	x0, x0, :lo12:multi_x
	ldr	w1, [x0, 36]
	adrp	x0, multi_x
	add	x0, x0, :lo12:multi_x
	ldr	w0, [x0, 28]
	add	w1, w1, w0
	ldr	w0, [x29, 56]
	cmp	w1, w0
	blt	.L1051
	ldr	w1, [x29, 64]
	mov	w0, 47823
	orr	w0, w1, w0
	str	w0, [x29, 64]
.L1051:
	adrp	x0, multi_o
	add	x0, x0, :lo12:multi_o
	ldr	w0, [x0, 36]
	cmp	w0, wzr
	blt	.L1052
	adrp	x0, multi_o
	add	x0, x0, :lo12:multi_o
	ldr	w0, [x0, 28]
	cmp	w0, wzr
	blt	.L1052
	adrp	x0, multi_o
	add	x0, x0, :lo12:multi_o
	ldr	w1, [x0, 36]
	adrp	x0, multi_o
	add	x0, x0, :lo12:multi_o
	ldr	w0, [x0, 28]
	add	w1, w1, w0
	ldr	w0, [x29, 56]
	cmp	w1, w0
	blt	.L1052
	ldr	w1, [x29, 64]
	mov	w0, 47062
	orr	w0, w1, w0
	str	w0, [x29, 64]
.L1052:
	ldr	w1, [x29, 64]
	mov	w0, 47823
	cmp	w1, w0
	beq	.L1053
	ldr	w1, [x29, 64]
	mov	w0, 47062
	cmp	w1, w0
	beq	.L1053
	str	wzr, [x29, 64]
.L1053:
	str	wzr, [x29, 60]
	b	.L1054
.L1055:
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	wzr, [x0, 4]
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 3
	add	x0, x1, x0
	str	wzr, [x0, 4]
	ldr	w0, [x29, 60]
	add	w0, w0, 1
	str	w0, [x29, 60]
.L1054:
	ldr	w0, [x29, 60]
	cmp	w0, 4
	ble	.L1055
	ldr	w0, [x29, 64]
	b	.L1073
.L1050:
	ldr	w0, [x29, 76]
	cmp	w0, 1
	beq	.L1057
	ldr	w0, [x29, 76]
	cmp	w0, 5
	ble	.L1058
.L1057:
	mov	w0, 0
	b	.L1073
.L1058:
	ldr	w1, [x29, 72]
	ldr	w0, [x29, 76]
	sdiv	w0, w1, w0
	str	w0, [x29, 72]
	ldr	w1, [x29, 68]
	ldr	w0, [x29, 76]
	sdiv	w0, w1, w0
	str	w0, [x29, 68]
	str	wzr, [x29, 32]
	ldr	w0, [x29, 72]
	uxth	w0, w0
	strh	w0, [x29, 34]
	ldr	w0, [x29, 68]
	uxth	w0, w0
	and	w0, w0, 4095
	uxth	w1, w0
	ldrh	w0, [x29, 32]
	bfi	w0, w1, 0, 12
	strh	w0, [x29, 32]
	str	wzr, [x29, 64]
	str	wzr, [x29, 60]
	b	.L1059
.L1062:
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	ldr	x1, [x29, 40]
	add	x0, x1, x0
	ldr	w0, [x0]
	cmp	w0, wzr
	bne	.L1060
	b	.L1061
.L1060:
	ldrsw	x0, [x29, 60]
	lsl	x0, x0, 2
	ldr	x1, [x29, 40]
	add	x0, x1, x0
	add	x1, x29, 32
	mov	w2, 1
	bl	GestureDistance
	mov	w1, w0
	ldr	w0, [x29, 64]
	add	w0, w0, w1
	str	w0, [x29, 64]
.L1061:
	ldr	w0, [x29, 60]
	add	w0, w0, 1
	str	w0, [x29, 60]
.L1059:
	ldr	w0, [x29, 60]
	cmp	w0, 9
	ble	.L1062
	ldr	w1, [x29, 64]
	ldr	w0, [x29, 76]
	sdiv	w0, w1, w0
	str	w0, [x29, 64]
	ldr	w0, [x29, 76]
	sub	w0, w0, #1
	str	w0, [x29, 76]
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	bne	.L1063
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	add	w1, w0, 1
	adrp	x0, multi_x
	add	x2, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0, 4]
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x29, 64]
	str	w1, [x0]
	b	.L1064
.L1063:
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	bge	.L1065
	b	.L1064
.L1065:
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x0]
	ldr	w0, [x29, 48]
	add	w1, w1, w0
	ldr	w0, [x29, 64]
	cmp	w1, w0
	bge	.L1066
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	mov	w1, -1
	str	w1, [x0, 4]
	b	.L1064
.L1066:
	ldr	w1, [x29, 64]
	ldr	w0, [x29, 52]
	add	w1, w1, w0
	adrp	x0, multi_x
	add	x2, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L1067
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	add	w1, w0, 1
	adrp	x0, multi_x
	add	x2, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0, 4]
	adrp	x0, multi_x
	add	x1, x0, :lo12:multi_x
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x29, 64]
	str	w1, [x0]
	b	.L1064
.L1067:
	nop
.L1064:
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	bne	.L1068
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	add	w1, w0, 1
	adrp	x0, multi_o
	add	x2, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0, 4]
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x29, 64]
	str	w1, [x0]
	b	.L1069
.L1068:
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	cmp	w0, wzr
	bge	.L1070
	b	.L1069
.L1070:
	ldr	w1, [x29, 64]
	ldr	w0, [x29, 48]
	add	w1, w1, w0
	adrp	x0, multi_o
	add	x2, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x2, x0
	ldr	w0, [x0]
	cmp	w1, w0
	bge	.L1071
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	mov	w1, -1
	str	w1, [x0, 4]
	b	.L1069
.L1071:
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x0]
	ldr	w0, [x29, 52]
	add	w1, w1, w0
	ldr	w0, [x29, 64]
	cmp	w1, w0
	bge	.L1072
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	add	w1, w0, 1
	adrp	x0, multi_o
	add	x2, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x2, x0
	str	w1, [x0, 4]
	adrp	x0, multi_o
	add	x1, x0, :lo12:multi_o
	ldrsw	x0, [x29, 76]
	lsl	x0, x0, 3
	add	x0, x1, x0
	ldr	w1, [x29, 64]
	str	w1, [x0]
	b	.L1069
.L1072:
	nop
.L1069:
	mov	w0, 0
.L1073:
	ldp	x29, x30, [sp], 80
	ret
	.size	GestureMulti, .-GestureMulti
	.align	2
	.global	gsl_GestureBuffer
	.type	gsl_GestureBuffer, %function
gsl_GestureBuffer:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	str	x0, [x29, 40]
	adrp	x0, gesture_num_last
	add	x0, x0, :lo12:gesture_num_last
	ldr	w0, [x0]
	cmp	w0, 255
	ble	.L1075
	adrp	x0, gesture_num_last
	add	x0, x0, :lo12:gesture_num_last
	mov	w1, 255
	str	w1, [x0]
.L1075:
	str	wzr, [x29, 60]
	b	.L1076
.L1077:
	ldrsw	x0, [x29, 60]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x19, x1, x0
	ldrsw	x0, [x29, 60]
	lsl	x1, x0, 2
	adrp	x0, config_static
	add	x0, x0, :lo12:config_static
	add	x0, x1, x0
	bl	ScreenResolution
	str	w0, [x19]
	ldr	w0, [x29, 60]
	add	w0, w0, 1
	str	w0, [x29, 60]
.L1076:
	adrp	x0, gesture_num_last
	add	x0, x0, :lo12:gesture_num_last
	ldr	w0, [x0]
	ldr	w1, [x29, 60]
	cmp	w1, w0
	blt	.L1077
	ldr	x0, [x29, 40]
	adrp	x1, config_static
	add	x1, x1, :lo12:config_static
	str	x1, [x0]
	adrp	x0, gesture_num_last
	add	x0, x0, :lo12:gesture_num_last
	ldr	w0, [x0]
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 64
	ret
	.size	gsl_GestureBuffer, .-gsl_GestureBuffer
	.local	save_dr.3292
	.comm	save_dr.3292,40,8
	.local	save_dn.3293
	.comm	save_dn.3293,40,8
	.ident	"GCC: (crosstool-NG linaro-1.13.1-4.9-2014.07 - Linaro GCC 4.9-2014.06) 4.9.1 20140529 (prerelease)"
