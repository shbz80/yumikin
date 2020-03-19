# yumikin
        kin_params_yumi = {}
        kin_params_yumi['urdf'] = '/home/shahbaz/Software/yumi_kinematics/yumikin/models/yumi_ABB_left.urdf'
        kin_params_yumi['base_link'] = 'world'
        # kin_params_yumi['end_link'] = 'left_tool0'
        kin_params_yumi['end_link'] = 'left_contact_point'
        kin_params_yumi['euler_string'] = 'sxyz'
        kin_params_yumi['goal'] = GOAL

        T = 200  # episode length

        yumiKin = YumiKinematics(kin_params_yumi)
