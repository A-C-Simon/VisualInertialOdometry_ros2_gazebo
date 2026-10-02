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
a=p.parse_args()
root=a.orb_root.resolve();out=a.output.resolve()
if not 0 <= a.keyframe_interval_s <= .5:
 p.error('Keyframe interval must be between 0 and 0.5 seconds')
if (a.preserve_inertial_origin or a.keyframe_interval_s) and out == (ROOT/'benchmark/build_orb_core').resolve():
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
          'original_library_sha256':hashlib.sha256((root/'lib/libORB_SLAM3.so').read_bytes()).hexdigest(),
          'flags':flags,'sources':{},'compile_commands':[]}
patch='';export_patch='';tracking_patch='';motion_patch='';objects={}
for name in ['Optimizer.cc','Settings.cc','System.cc','Tracking.cc','LocalMapping.cc']:
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
  if a.keyframe_interval_s:
   marker='    if(mbOnlyTracking)\n        return false;'
   assert new.count(marker)==1
   new=new.replace(marker,'''    // Diagnostic trial: retain frame tracking while avoiding redundant
    // high-frequency keyframes when stereo-inertial tracking is healthy.
    // Preserve urgent insertion with weak tracking or a tracking loss.
    if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
        mpAtlas->GetCurrentMap()->isImuInitialized() && mpLastKeyFrame &&
        mState == OK && mnMatchesInliers >= 50 &&
        mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp < '''+
        format(a.keyframe_interval_s,'.17g')+''')
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
 else:
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
 manifest['sources'][name]={'original_sha256':hashlib.sha256(old.encode()).hexdigest(),
                           'patched_sha256':hashlib.sha256(new.encode()).hexdigest()}
 source_patch=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/src/'+name,tofile='b/src/'+name))
 if name=='Optimizer.cc' and a.preserve_inertial_origin:
  # Keep the default local-BA patch independent of the experimental change.
  origin_patch=(ROOT/'benchmark/patches/orb_inertial_origin.patch').read_text()
  manifest['inertial_origin_patch_sha256']=hashlib.sha256(origin_patch.encode()).hexdigest()
  patch+=''.join(difflib.unified_diff(old.splitlines(True),baseline_new.splitlines(True),
                                    fromfile='a/src/'+name,tofile='b/src/'+name))
 elif name=='System.cc':export_patch+=source_patch
 elif name=='Tracking.cc':
  if a.keyframe_interval_s:
   (out/'orb_keyframe_interval.patch').write_text(source_patch)
  else:tracking_patch+=source_patch
 elif name=='LocalMapping.cc':motion_patch+=source_patch
 else:patch+=source_patch
 obj=out/(name+'.o');objects['CMakeFiles/ORB_SLAM3.dir/src/'+name+'.o']=str(obj)
 command=['/usr/bin/c++']+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+flags['CXX_FLAGS']+['-c',str(patched),'-o',str(obj)]
 manifest['compile_commands'].append(command)
(ROOT/'benchmark/patches/orb_local_ba_window.patch').write_text(patch)
(ROOT/'benchmark/patches/orb_safe_trajectory_export.patch').write_text(export_patch)
if not a.keyframe_interval_s:
 (ROOT/'benchmark/patches/orb_tracking_reset_window.patch').write_text(tracking_patch)
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
