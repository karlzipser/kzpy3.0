"""



"""

from kzpy3.vis import *


class Bag_File:
    def __init__(self, path, max_requests):
        self.path = path
        self.img_dic = None
        self.timestamps = None
        self.max_requests = max_requests
        self.request_ctr = 0

    def reset(self):
        self.request_ctr = 0

    def get_data(self, left_image_bound_to_data, target_topics, num_data_steps, num_frames):
            if self.request_ctr >= self.max_requests:
                return None
            if self.img_dic == None:
                if os.path.getsize(self.path) < 20000000:
                    return None
                #print 'Bag_File: loading ' + self.path
                self.img_dic = load_obj(self.path)
                self.timestamps = sorted(self.img_dic['left'].keys())
                self.binned_timestamp_nums = [[],[]]
                """
                Binning timestamps. Most of the time the car drives straight.
                If we sample the timestamps uniformly, we will not get many examples
                of turning, proportionally. Therefore, I bin timestamps. I use only two
                bins, mostly-straight and turning. Having done this, we can request a
                random choice within either category.
                """
                for i in range(len(self.timestamps)-num_data_steps):
                    t = self.timestamps[i+num_data_steps]
                    if t in left_image_bound_to_data:
                        if left_image_bound_to_data[t]['state_one_steps'] > num_data_steps:
                            steer = left_image_bound_to_data[t]['steer']
                            if steer < 43 or steer > 55:
                                self.binned_timestamp_nums[0].append(i)
                            else:
                                self.binned_timestamp_nums[1].append(i)
            #print((len(self.binned_timestamp_nums[0]),len(self.binned_timestamp_nums[1])))
            """
            Sometimes a bin is empty, so we need to check for this while making a
            random selection.
            """
            if len(self.binned_timestamp_nums[0]) > 0 and len(self.binned_timestamp_nums[1]) > 0:
                timestamp_num = random.choice(self.binned_timestamp_nums[np.random.randint(len(self.binned_timestamp_nums))])
            elif len(self.binned_timestamp_nums[0]) > 0:
                timestamp_num = random.choice(self.binned_timestamp_nums[0])
            elif len(self.binned_timestamp_nums[1]) > 0:
                timestamp_num = random.choice(self.binned_timestamp_nums[1])
            else:
                return None

            t = self.timestamps[timestamp_num]
            """ Here is our timestamp."""

            data_dic = {}

            if t in left_image_bound_to_data:
                if left_image_bound_to_data[t]['state_one_steps'] > num_data_steps:
                    if timestamp_num + num_data_steps <= len(self.timestamps):
                        for topic in target_topics:
                            target_data = []
                            left_list = []
                            right_list = []
                            fctr = 0
                            for tn in range(timestamp_num,timestamp_num+num_data_steps):
                                data = "NO DATA"
                                if topic in left_image_bound_to_data[self.timestamps[tn]]:
                                    data = left_image_bound_to_data[self.timestamps[tn]][topic]
                                target_data.append(data)
                                if fctr < num_frames:
                                    left_list.append(self.img_dic['left'][self.timestamps[tn]])
                                    right_list.append(self.img_dic['right'][left_image_bound_to_data[self.timestamps[tn]]['right_image']])
                                    fctr += 1
                            data_dic[topic] = target_data
                        data_dic['left'] = left_list
                        data_dic['right'] = right_list
            else:
                return None
            if len(data_dic) == 0:
                return None

            # assert that data_dic holds what it is supposed to hold.
            print self.path
            print data_dic.keys()
            for topic in target_topics:
                assert type(data_dic[topic]) == list
                assert len(data_dic[topic]) == num_data_steps
            for side in ['left','right']:
                assert type(data_dic[side]) == list
                assert len(data_dic[side]) == num_frames
                for i in range(num_frames):
                    assert type(data_dic[side][i]) == np.ndarray
                    assert shape(data_dic[side][i]) == (94, 168)

            self.request_ctr += 1
            #print d2s("Bag_File::request_ctr =",self.request_ctr)
            return data_dic


class Bag_Folder:
    def __init__(self, path, max_requests, max_subrequests):
        self.files = sorted(glob.glob(opj(path,'.preprocessed','*.bag.pkl')))
        file_path = opj(path,'.preprocessed','left_image_bound_to_data')
        print "Bag_Folder: loading "+file_path+'.pkl'
        self.left_image_bound_to_data = load_obj(file_path)
        self.bag_file = None
        self.request_ctr = 0
        self.max_requests = max_requests
        self.max_subrequests = max_subrequests
        self.bag_files_dic = {}
        # The state_one_steps were forund in preprocess_bag_data.py, but I redo it here to get state 3.
        """
        -- state_one_steps --
        Data at a given timestamp may be valid or invalid with respect to our data needs.
        For example, state 2 is by definition invalid, while state 1 is by definition valid.
        We also want to know, at any given timestamp, how many valid timestamps there are in a continuous
        sequence in the future. That is what we are figuring out here.
        A problem is that state number is not necessarily enough. For example, the car could
        be in state 1 but not be moving, or it could be in state 1 an be being carried by hand
        because there was a mistake in changing states -- but the lack of a motor signal could catch this.
        """
        timestamps = sorted(self.left_image_bound_to_data.keys())
        state_one_steps = 0
        for i in range(len(timestamps)-1,-1,-1):
            state = self.left_image_bound_to_data[timestamps[i]]['state']
            motor = self.left_image_bound_to_data[timestamps[i]]['motor']
            if state in [1,3,5,6,7]: #== 1.0 or state == 3.0:
                if motor > 51: # i.e., there must be at least a slight forward motor command 
                    state_one_steps += 1
                else:
                    state_one_steps = 0
            else:
                state_one_steps = 0
            self.left_image_bound_to_data[timestamps[i]]['state_one_steps'] = state_one_steps

    def reset(self):
        self.request_ctr = 0

    def get_data(self, target_topics, num_data_steps, num_frames):
        if len(self.files) == 0:
            return None
        assert len(self.files) > 0
        #print 'Bag_Folder::get_data'
        if self.request_ctr >= self.max_requests:
            return None
        if self.bag_file == None:
            b = random.choice(self.files)
            if b not in self.bag_files_dic:
                if '/Users/' not in home_path: # OSX doesn't have the memory() function that linux has.
                    m=memory()
                    if m['free']/(1.0*m['total']) < 0.15:
                        if len(self.bag_files_dic.keys()) > 0:
                            rc = random.choice(self.bag_files_dic.keys())
                            #print "Bag_Folder: deleting " + rc + " *****************************************"
                            del self.bag_files_dic[rc]
                self.bag_files_dic[b] = Bag_File(b, self.max_subrequests)
            self.bag_file = self.bag_files_dic[b]
            self.bag_file.reset()
            #self.bag_file = Bag_File(b, self.max_subrequests)
        if True:#try:
            data = self.bag_file.get_data(self.left_image_bound_to_data, target_topics, num_data_steps, num_frames)
            if not data == None:
                data['bag_filename'] = self.bag_file.path
            #print b
        else: #except Exception, e:
            #print e 
            #print "Bag_Folder ***************************************"
            data = None

        if data == None:
            self.bag_file = None
            return self.get_data(target_topics, num_data_steps, num_frames)
        self.request_ctr += 1
        #print d2s("Bag_Folder::request_ctr =",self.request_ctr)
        return data



class Bair_Car_Data:
    """ """
    def __init__(self, path, max_requests, max_subrequests):
        bag_folder_paths = sorted(glob.glob(opj(path,'*')))
        self.bag_folders_weighted = []
        for f in bag_folder_paths:
            
            n = len(gg(opj(f,'.preprocessed','*.bag.pkl')))
            m = len(gg(opj(f,'.preprocessed','left*')))
            print (n,m,f)
            if n > 0 and m > 0:
                for i in range(max(n/10,1)):
                    self.bag_folders_weighted.append(f)
        #print self.bag_folders_weighted
        #time.sleep(60)
        self.bag_folder = None
        self.max_requests = max_requests
        self.max_subrequests = max_subrequests
        self.bag_folders_dic = {}

    def get_data(self, target_topics, num_data_steps, num_frames):
        #print 'Bair_Car_Data::get_data'
        if True:#try:
            if self.bag_folder == None:
                b = random.choice(self.bag_folders_weighted)
                if b not in self.bag_folders_dic:
                    self.bag_folders_dic[b] = Bag_Folder(b, self.max_requests, self.max_subrequests)
                    print d2s("len(self.bag_folders_dic) =",len(self.bag_folders_dic))
                self.bag_folder = self.bag_folders_dic[b]
                self.bag_folder.reset()

            data = self.bag_folder.get_data(target_topics, num_data_steps, num_frames)
        else: #except Exception, e:
            #print e 
            #print "Bair_Car_Data ***************************************"
            data = None

        if data == None:
            self.bag_folder = None
            return self.get_data(target_topics, num_data_steps, num_frames)
        return data


