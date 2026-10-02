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
p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
root=a.orb_root.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
flags_file=root/'build/CMakeFiles/ORB_SLAM3.dir/flags.make'
flags={}
for line in flags_file.read_text().splitlines():
 if line.startswith('CXX_'):
  key,value=line.split('=',1);flags[key.strip()]=shlex.split(value.strip())
manifest={'original_library_sha256':hashlib.sha256((root/'lib/libORB_SLAM3.so').read_bytes()).hexdigest(),
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
 patched=out/name;patched.write_text(new)
 manifest['sources'][name]={'original_sha256':hashlib.sha256(old.encode()).hexdigest(),
                           'patched_sha256':hashlib.sha256(new.encode()).hexdigest()}
 source_patch=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/src/'+name,tofile='b/src/'+name))
 if name=='System.cc':export_patch+=source_patch
 elif name=='Tracking.cc':tracking_patch+=source_patch
 elif name=='LocalMapping.cc':motion_patch+=source_patch
 else:patch+=source_patch
 obj=out/(name+'.o');objects['CMakeFiles/ORB_SLAM3.dir/src/'+name+'.o']=str(obj)
 command=['/usr/bin/c++']+flags['CXX_DEFINES']+flags['CXX_INCLUDES']+flags['CXX_FLAGS']+['-c',str(patched),'-o',str(obj)]
 manifest['compile_commands'].append(command)
(ROOT/'benchmark/patches/orb_local_ba_window.patch').write_text(patch)
(ROOT/'benchmark/patches/orb_safe_trajectory_export.patch').write_text(export_patch)
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
