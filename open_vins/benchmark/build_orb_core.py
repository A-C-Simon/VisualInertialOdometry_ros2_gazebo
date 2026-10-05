#!/usr/bin/env python3
"""Build an isolated ORB core variant from a compatible existing CMake build.
Reuses unchanged object files; recompiles only patched sources. Never writes to
ORB_SLAM3_ROOT. The original flags, link command and hashes are recorded.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--orb-root',type=Path,default=Path('/home/ac/ORB_SLAM3'))
p.add_argument('--output',type=Path,default=ROOT/'benchmark/build_orb_core')
p.add_argument('--prepare-only',action='store_true')
p.add_argument('--preserve-inertial-origin',action='store_true',
               help='Experimental: preserve the translation origin through full inertial BA')
p.add_argument('--check-translation-invariance',type=Path,metavar='ORB_SETTINGS',
               help='Build and run the native visual/inertial edge check with these settings')
p.add_argument('--keyframe-interval-s',type=float,default=0.,
               help='Experimental minimum interval for healthy initialized stereo-inertial keyframes (0 to 0.5 s)')
p.add_argument('--refined-keyframe-interval-s',type=float,default=0.,
               help='Experimental interval after second inertial refinement; 0 retains the initial interval')
p.add_argument('--motion-gated-initialization',action='store_true',
               help='Experimental: require measured translation before inertial initialization and wait during quiet intervals')
p.add_argument('--fast-stereo-patches',action='store_true',
               help='Experimental: retain exact stereo L1 distances while reducing temporary allocations')
p.add_argument('--fast-gaussian',action='store_true',
               help='Experimental: use equivalent fixed-point descriptor blur on OpenCV 4.5.4')
p.add_argument('--reuse-pyramid',action='store_true',
               help='Experimental: retain image pyramid allocations when dimensions match')
p.add_argument('--packed-vocabulary',action='store_true',
               help='Experimental: parse valid ORB vocabulary nodes into shared descriptor storage')
p.add_argument('--profile-cpu',action='store_true',
               help='Diagnostic only: collect inclusive per-stage thread CPU with ORB_PROFILE_OUTPUT')
a=p.parse_args()
root=a.orb_root.resolve();out=a.output.resolve()
if not 0 <= a.keyframe_interval_s <= .5:
 p.error('Keyframe interval must be between 0 and 0.5 seconds')
if not 0 <= a.refined_keyframe_interval_s <= .5:
 p.error('Refined keyframe interval must be between 0 and 0.5 seconds')
if a.refined_keyframe_interval_s and a.refined_keyframe_interval_s < a.keyframe_interval_s:
 p.error('Refined keyframe interval must not shorten the initial interval')
if (a.preserve_inertial_origin or a.keyframe_interval_s or a.refined_keyframe_interval_s or a.motion_gated_initialization or a.fast_stereo_patches or a.fast_gaussian or a.reuse_pyramid or a.packed_vocabulary or a.profile_cpu) and out == (ROOT/'benchmark/build_orb_core').resolve():
 p.error('Use a separate --output directory for experimental core changes')
if a.check_translation_invariance and (a.prepare_only or not a.check_translation_invariance.is_file()):
 p.error('Translation check requires an existing settings file and a compiled build')
out.mkdir(parents=True,exist_ok=True)
flags_file=root/'build/CMakeFiles/ORB_SLAM3.dir/flags.make'
flags={}
for line in flags_file.read_text().splitlines():
 if line.startswith('CXX_'):
  key,value=line.split('=',1);flags[key.strip()]=shlex.split(value.strip())
manifest={'preserve_inertial_origin':a.preserve_inertial_origin,
          'keyframe_interval_s':a.keyframe_interval_s,
          'refined_keyframe_interval_s':a.refined_keyframe_interval_s,
          'motion_gated_initialization':a.motion_gated_initialization,
          'fast_stereo_patches':a.fast_stereo_patches,
          'fast_gaussian':a.fast_gaussian,
          'reuse_pyramid':a.reuse_pyramid,
          'packed_vocabulary':a.packed_vocabulary,
          'profile_cpu':a.profile_cpu,
          'original_library_sha256':hashlib.sha256((root/'lib/libORB_SLAM3.so').read_bytes()).hexdigest(),
          'flags':flags,'sources':{},'compile_commands':[]}
patch='';export_patch='';tracking_patch='';motion_patch='';objects={}
source_names=['Optimizer.cc','Settings.cc','System.cc','Tracking.cc','LocalMapping.cc','Frame.cc','LoopClosing.cc']
if a.profile_cpu or a.fast_gaussian or a.reuse_pyramid:
 source_names.append('ORBextractor.cc')
if a.profile_cpu:
 profile_header=ROOT/'benchmark/orb_cpu_profile.hpp'
 (out/profile_header.name).write_bytes(profile_header.read_bytes())
 manifest['cpu_profile_header_sha256']=hashlib.sha256(profile_header.read_bytes()).hexdigest()
 manifest['cpu_profile_scopes']={}
for name in source_names:
 source=root/'src'/name;old=source.read_text();new=old
 if name=='Optimizer.cc':
  new=new.replace('#include <complex>','#include <complex>\n#include <cstdlib>')
  marker='    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap()-2,maxOpt);'
  assert new.count(marker)==1
  new=new.replace(marker,'''    // Keep initialization/global inertial BA intact. Optionally bound the
    // number of locally optimized keyframe states once local BA is invoked.
    static const int window_cap = []() {
        const char* value = std::getenv("ORB_LOCAL_BA_WINDOW");
        if (!value) return 25;
        char* end = nullptr;
        const long cap = std::strtol(value, &end, 10);
        if (*end != '\\0' || cap < 6 || cap > 25) return 25;
        std::cout << "Local inertial BA window cap: " << cap << std::endl;
        return static_cast<int>(cap);
    }();
    maxOpt = std::min(maxOpt, window_cap);
''' + marker)
 elif name=='Settings.cc':
  start=new.index('void Settings::readCamera2(');end=new.index('void Settings::readImageInfo(',start)
  section=new[start:end]
  marker='        else if(cameraType_ == KannalaBrandt){'
  assert section.count(marker)==1
  section=section.replace(marker,'''        else if(cameraType_ == Rectified){
            // Rectified stereo uses the common pinhole model. Initialize the
            // second camera too: settings diagnostics dereference this object.
            for (size_t i = 0; i < 4; ++i)
                vCalibration.push_back(calibration1_->getParameter(i));
            calibration2_ = new Pinhole(vCalibration);
            originalCalib2_ = new Pinhole(vCalibration);
        }
''' +marker)
  new=new[:start]+section+new[end:]
 elif name=='Tracking.cc':
  marker='''    if(settings){
        newParameterLoader(settings);
    }'''
  assert new.count(marker)==1
  new=new.replace(marker,'''    if(settings){
        newParameterLoader(settings);
        // The legacy loader sets this; the version 1.0 loader omitted it.
        // Tracking and inertial optimization must use a deterministic window.
        mnFramesToResetIMU = mMaxFrames;
    }''')
  if a.keyframe_interval_s or a.refined_keyframe_interval_s:
   interval=format(a.keyframe_interval_s,'.17g')
   if a.refined_keyframe_interval_s:
    interval='(mpAtlas->GetCurrentMap()->GetIniertialBA2() ? '+format(a.refined_keyframe_interval_s,'.17g')+' : '+interval+')'
   marker='    if(mbOnlyTracking)\n        return false;'
   assert new.count(marker)==1
   new=new.replace(marker,'''    // Diagnostic trial: retain frame tracking while avoiding redundant
    // high-frequency keyframes when stereo-inertial tracking is healthy.
    // Preserve urgent insertion with weak tracking or a tracking loss.
    if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
        mpAtlas->GetCurrentMap()->isImuInitialized() && mpLastKeyFrame &&
        mState == OK && mnMatchesInliers >= 50 &&
        mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp < '''+interval+''')
        return false;

'''+marker)
 elif name=='LocalMapping.cc':
  marker='''                                cout << "Not enough motion for initializing. Reseting..." << endl;'''
  assert new.count(marker)==1
  new=new.replace(marker,'''                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                cout << "IMU_MOTION_RESET timestamp=" << mpCurrentKeyFrame->mTimeStamp
                                     << " distance_m=" << dist << " motion_time_s=" << mTinit
                                     << " keyframes=" << mpCurrentKeyFrame->GetMap()->KeyFramesInMap()
                                     << " refinement1=" << mpCurrentKeyFrame->GetMap()->GetIniertialBA1()
                                     << " refinement2=" << mpCurrentKeyFrame->GetMap()->GetIniertialBA2()
                                     << endl;''')
 elif name=='LoopClosing.cc':
  marker='    mpLastCurrentKF = static_cast<KeyFrame*>(NULL);'
  assert new.count(marker)==1
  new=new.replace(marker,'    mpCurrentKF = nullptr;\n'+marker)
  marker='''void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);'''
  assert new.count(marker)==1
  new=new.replace(marker,'''void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    // Disabled place recognition returns before consuming this queue. Do
    // not enqueue work that would read an uninitialized current keyframe.
    if (!mbActiveLC) return;
    unique_lock<mutex> lock(mMutexLoopQueue);''')
 elif name=='Frame.cc':
  # Stereo matching reads mb before the constructor body computes static fx.
  # Use this frame's intrinsics rather than uninitialized object storage.
  start=new.index('Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,')
  end=new.index('Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,',start)
  section=new[start:end]
  marker='mbf(bf), mThDepth(thDepth)'
  assert section.count(marker)==1
  section=section.replace(marker,'mbf(bf), mb(bf / K.at<float>(0,0)), mThDepth(thDepth)')
  new=new[:start]+section+new[end:]
 elif name=='System.cc':
  # An atlas emptied by initialization resets has no map with keyframes.
  # Both exporters otherwise dereference an uninitialized map pointer.
  for begin,end in [('void System::SaveTrajectoryEuRoC(const string &filename)',
                     'void System::SaveTrajectoryEuRoC(const string &filename, Map* pMap)'),
                    ('void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)',
                     'void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap)')]:
   start=new.index(begin);finish=new.index(end,start);section=new[start:finish]
   assert section.count('Map* pBiggerMap;')==1
   section=section.replace('Map* pBiggerMap;','Map* pBiggerMap = nullptr;')
   if begin.startswith('void System::SaveTrajectoryEuRoC('):
    marker='    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();'
    assert section.count(marker)==1
    section=section.replace(marker,'''    if (!pBiggerMap)
    {
        cout << "No map with keyframes; trajectory export skipped." << endl;
        return;
    }

'''+marker)
   new=new[:start]+section+new[finish:]
 baseline_new=new
 patched=out/name;patched.write_text(new)
 if name=='Optimizer.cc' and a.preserve_inertial_origin:
  subprocess.run(['patch','--silent','--fuzz=0',str(patched),
                  str(ROOT/'benchmark/patches/orb_inertial_origin.patch')],check=True)
  new=patched.read_text()
 if name=='LocalMapping.cc' and a.motion_gated_initialization:
  policy=ROOT/'benchmark/patches/orb_motion_gated_initialization.patch'
  subprocess.run(['patch','--silent','--fuzz=0',str(patched),str(policy)],check=True)
  new=patched.read_text()
  manifest['motion_gate_patch_sha256']=hashlib.sha256(policy.read_bytes()).hexdigest()
 if name=='Frame.cc':
  baseline_patch=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
                          fromfile='a/src/Frame.cc',tofile='b/src/Frame.cc'))
  (ROOT/'benchmark/patches/orb_stereo_baseline_initialization.patch').write_text(baseline_patch)
  if a.fast_stereo_patches:
   section=new[new.index('void Frame::ComputeStereoMatches()'):new.index('void Frame::ComputeStereoFromRGBD(')]
   assert 'const int w = 5;' in section and 'const int L = 5;' in section
   policy=ROOT/'benchmark/patches/orb_stereo_patch_cost.patch'
   subprocess.run(['patch','--silent','--fuzz=0',str(patched),str(policy)],check=True)
   new=patched.read_text()
   header=ROOT/'benchmark/stereo_patch_distance.hpp'
   (out/header.name).write_bytes(header.read_bytes())
   manifest['stereo_patch_sha256']=hashlib.sha256(policy.read_bytes()).hexdigest()
   manifest['stereo_patch_header_sha256']=hashlib.sha256(header.read_bytes()).hexdigest()
 if name=='System.cc' and a.packed_vocabulary:
  header=ROOT/'benchmark/orb_packed_vocabulary.hpp'
  (out/header.name).write_bytes(header.read_bytes())
  manifest['packed_vocabulary_header_sha256']=hashlib.sha256(header.read_bytes()).hexdigest()
  new='#include "orb_packed_vocabulary.hpp"\n'+new
  (out/'orb_packed_vocabulary.patch').write_text(''.join(difflib.unified_diff(
      patched.read_text().splitlines(True),new.splitlines(True),
      fromfile='a/src/System.cc',tofile='b/src/System.cc')))
  patched.write_text(new)
 if name=='ORBextractor.cc' and a.fast_gaussian:
  marker='''            Mat workingMat = mvImagePyramid[level].clone();
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);'''
  assert new.count(marker)==1
  replacement='''            Mat workingMat;
            orb_fast::gaussian7(mvImagePyramid[level], workingMat);'''
  new='#include "orb_gaussian7.hpp"\n'+new.replace(marker,replacement)
  header=ROOT/'benchmark/orb_gaussian7.hpp'
  (out/header.name).write_bytes(header.read_bytes())
  manifest['gaussian_header_sha256']=hashlib.sha256(header.read_bytes()).hexdigest()
  gaussian_patch=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
                           fromfile='a/src/ORBextractor.cc',tofile='b/src/ORBextractor.cc'))
  (out/'orb_gaussian.patch').write_text(gaussian_patch)
  patched.write_text(new)
 if name=='ORBextractor.cc' and a.reuse_pyramid:
  before_reuse=new
  marker='            Mat temp(wholeSize, image.type()), masktemp;'
  assert new.count(marker)==1
  new=new.replace(marker,'''            // Reuse the existing pyramid allocation without changing the
            // extractor layout or the image/border computation.
            Mat temp;
            if (mvImagePyramid[level].size() == sz &&
                mvImagePyramid[level].type() == image.type())
            {
                Size allocatedSize;
                Point offset;
                mvImagePyramid[level].locateROI(allocatedSize, offset);
                if (allocatedSize == wholeSize &&
                    offset == Point(EDGE_THRESHOLD, EDGE_THRESHOLD))
                {
                    temp = mvImagePyramid[level];
                    temp.adjustROI(EDGE_THRESHOLD, EDGE_THRESHOLD,
                                   EDGE_THRESHOLD, EDGE_THRESHOLD);
                }
            }
            if (temp.empty()) temp.create(wholeSize, image.type());''')
  (out/'orb_pyramid_reuse.patch').write_text(''.join(difflib.unified_diff(
      before_reuse.splitlines(True),
      new.splitlines(True),fromfile='a/src/ORBextractor.cc',tofile='b/src/ORBextractor.cc')))
  patched.write_text(new)
 policy_new=new
 if a.profile_cpu:
  scope_names={
   'System.cc':['TrackStereo'],
   'Frame.cc':['ExtractORB','ComputeStereoMatches','ComputeBoW','AssignFeaturesToGrid'],
   'Tracking.cc':['GrabImageStereo','Track','PreintegrateIMU','TrackLocalMap','SearchLocalPoints','TrackWithMotionModel','TrackReferenceKeyFrame','UpdateLocalMap','CreateNewKeyFrame'],
   'LocalMapping.cc':['Run','ProcessNewKeyFrame','CreateNewMapPoints','SearchInNeighbors','MapPointCulling','KeyFrameCulling','InitializeIMU','ScaleRefinement'],
   'Optimizer.cc':['PoseOptimization','PoseInertialOptimizationLastFrame','PoseInertialOptimizationLastKeyFrame','LocalInertialBA','LocalBundleAdjustment','FullInertialBA','InertialOptimization'],
   'ORBextractor.cc':['ComputePyramid','ComputeKeyPointsOctTree','DistributeOctTree']}
  if name in scope_names:
   cls=name[:-3]
   new='#include "orb_cpu_profile.hpp"\n'+new
   for function in scope_names[name]:
    label=cls+'::'+function
    pattern=r'(^[^\n;{}]*\b'+re.escape(label)+r'\s*\([^;{}]*\)\s*\n\s*\{)'
    new,count=re.subn(pattern,lambda m:m[0]+'\n    ORB_CPU_SCOPE('+json.dumps(label)+');',new,flags=re.M)
    assert count>0,label
    manifest['cpu_profile_scopes'][label]=count
   if name=='ORBextractor.cc':
    for function in ['computeOrientation','computeDescriptors']:
     pattern=r'(^\s*static void '+function+r'\s*\([^;{}]*\)\s*\n\s*\{)'
     label='ORBextractor::'+function
     new,count=re.subn(pattern,lambda m:m[0]+'\n        ORB_CPU_SCOPE('+json.dumps(label)+');',new,flags=re.M)
     assert count==1,label
     manifest['cpu_profile_scopes'][label]=count
    marker=('            orb_fast::gaussian7(mvImagePyramid[level], workingMat);' if a.fast_gaussian else
            '            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);')
    assert new.count(marker)==1
    new=new.replace(marker,'            { ORB_CPU_SCOPE("ORBextractor::GaussianBlur");\n'+marker+'\n            }')
    manifest['cpu_profile_scopes']['ORBextractor::GaussianBlur']=1
   patched.write_text(new)
 manifest['sources'][name]={'original_sha256':hashlib.sha256(old.encode()).hexdigest(),
                           'patched_sha256':hashlib.sha256(new.encode()).hexdigest()}
 source_patch=''.join(difflib.unified_diff(old.splitlines(True),policy_new.splitlines(True),fromfile='a/src/'+name,tofile='b/src/'+name))
 if name=='Optimizer.cc' and a.preserve_inertial_origin:
  # Keep the default local-BA patch independent of the experimental change.
  origin_patch=(ROOT/'benchmark/patches/orb_inertial_origin.patch').read_text()
  manifest['inertial_origin_patch_sha256']=hashlib.sha256(origin_patch.encode()).hexdigest()
  patch+=''.join(difflib.unified_diff(old.splitlines(True),baseline_new.splitlines(True),
                                    fromfile='a/src/'+name,tofile='b/src/'+name))
 elif name=='System.cc':
  # Keep the ordinary export fix independent of optional loader experiments.
  export_patch+=''.join(difflib.unified_diff(old.splitlines(True),baseline_new.splitlines(True),
                         fromfile='a/src/'+name,tofile='b/src/'+name))
 elif name=='Tracking.cc':
  if a.keyframe_interval_s or a.refined_keyframe_interval_s:
   (out/'orb_keyframe_interval.patch').write_text(source_patch)
  else:tracking_patch+=source_patch
 elif name=='LocalMapping.cc':
  if a.motion_gated_initialization:
   (out/'orb_motion_gate.patch').write_text(source_patch)
  else:motion_patch+=source_patch
 elif name=='LoopClosing.cc':
  (ROOT/'benchmark/patches/orb_disabled_loop_queue.patch').write_text(source_patch)
 elif name not in ('Frame.cc','ORBextractor.cc'):patch+=source_patch
 obj=out/(name+'.o');objects['CMakeFiles/ORB_SLAM3.dir/src/'+name+'.o']=str(obj)
 command=['/usr/bin/c++']+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+flags['CXX_FLAGS']+['-c',str(patched),'-o',str(obj)]
 manifest['compile_commands'].append(command)
(ROOT/'benchmark/patches/orb_local_ba_window.patch').write_text(patch)
(ROOT/'benchmark/patches/orb_safe_trajectory_export.patch').write_text(export_patch)
if not (a.keyframe_interval_s or a.refined_keyframe_interval_s):
 (ROOT/'benchmark/patches/orb_tracking_reset_window.patch').write_text(tracking_patch)
if not a.motion_gated_initialization:
 (ROOT/'benchmark/patches/orb_initialization_diagnostics.patch').write_text(motion_patch)
link=shlex.split((root/'build/CMakeFiles/ORB_SLAM3.dir/link.txt').read_text())
link[link.index('-o')+1]=str(out/'libORB_SLAM3.so')
link=[objects.get(arg,arg) for arg in link];manifest['link_command']=link
(out/'build_manifest.json').write_text(json.dumps(manifest,indent=2))
if not a.prepare_only:
 for command in manifest['compile_commands']:subprocess.run(command,cwd=root/'build',check=True)
 subprocess.run(link,cwd=root/'build',check=True)
 print('Built isolated library:',out/'libORB_SLAM3.so')
 if a.check_translation_invariance:
  test_source=ROOT/'benchmark/orb_translation_invariance.cc'
  test_object=out/'orb_translation_invariance.o'
  test_binary=out/'orb_translation_invariance'
  compile_test=['/usr/bin/c++']+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+[
      '-O0','-std=c++14','-Wno-deprecated-declarations','-c',str(test_source),'-o',str(test_object)]
  dependencies=[x for x in link if x.startswith('-l') or
      (not x.startswith('-') and (x.endswith('.so') or '.so.' in x) and x!=str(out/'libORB_SLAM3.so'))]
  link_test=['/usr/bin/c++',str(test_object),'-o',str(test_binary),str(out/'libORB_SLAM3.so')]+dependencies+[
      '-Wl,-rpath,'+str(out),'-Wl,-rpath,'+str(root/'Thirdparty/g2o/lib'),
      '-Wl,-rpath,'+str(root/'Thirdparty/DBoW2/lib')]
  with (out/'translation_check_build.log').open('w') as build_log:
   for command in [compile_test,link_test]:
    subprocess.run(command,cwd=root/'build',stdout=build_log,stderr=subprocess.STDOUT,check=True)
  settings=a.check_translation_invariance.resolve()
  result=subprocess.run([str(test_binary),str(settings)],cwd=root/'build',text=True,
                        stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
  (out/'translation_check.txt').write_text(result.stdout)
  manifest['translation_check']={'compile':compile_test,'link':link_test,
      'settings':str(settings),'settings_sha256':hashlib.sha256(settings.read_bytes()).hexdigest(),
      'source_sha256':hashlib.sha256(test_source.read_bytes()).hexdigest(),
      'exit_code':result.returncode,'output':result.stdout}
  (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  print(result.stdout,end='')
  result.check_returncode()
