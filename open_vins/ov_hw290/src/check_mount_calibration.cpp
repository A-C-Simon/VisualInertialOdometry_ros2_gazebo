#include <opencv2/core.hpp>
#include <openssl/evp.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

std::string sha256(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot read calibration: " + path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("Cannot initialize SHA256");
  char buffer[8192];
  while (input.read(buffer, sizeof buffer) || input.gcount())
    if (EVP_DigestUpdate(ctx.get(), buffer, input.gcount()) != 1)
      throw std::runtime_error("Cannot hash calibration");
  if (input.bad()) throw std::runtime_error("Calibration read failed");
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest, &length) != 1)
    throw std::runtime_error("Cannot finalize calibration hash");
  std::ostringstream result;
  for (unsigned i = 0; i < length; ++i)
    result << std::hex << std::setfill('0') << std::setw(2) << unsigned(digest[i]);
  return result.str();
}

int main(int argc, char** argv) {
  const bool candidate = argc == 5 && std::string(argv[4]) == "--candidate";
  if (argc != 4 && !candidate) {
    std::cerr << "Usage: check_mount_calibration CURRENT_MOUNT ESTIMATOR_CONFIG STEREO_CALIBRATION [--candidate]\n";
    return 2;
  }
  try {
    cv::FileStorage mount(argv[1], cv::FileStorage::READ);
    if (!mount.isOpened()) throw std::runtime_error("Current mount metadata is unavailable");
    std::string id, status, chain_hash, stereo_hash, reason;
    mount["mount_id"] >> id; mount["status"] >> status;
    mount["validated_chain_sha256"] >> chain_hash;
    mount["validated_stereo_sha256"] >> stereo_hash; mount["reason"] >> reason;
    const bool approved_status = status == "validated" ||
        (candidate && status == "fitted_pending_validation");
    if (id.empty() || !approved_status || chain_hash.size() != 64 || stereo_hash.size() != 64)
      throw std::runtime_error("Mount " + id + " needs calibration. " + reason);
    cv::FileStorage config(argv[2], cv::FileStorage::READ);
    if (!config.isOpened()) throw std::runtime_error("Estimator config is unavailable");
    std::string relative_chain; config["relative_config_imucam"] >> relative_chain;
    if (relative_chain.empty() || fs::path(relative_chain).is_absolute())
      throw std::runtime_error("Expected a relative camera/IMU chain path");
    const auto chain = fs::path(argv[2]).parent_path() / relative_chain;
    if (sha256(chain) != chain_hash || sha256(argv[3]) != stereo_hash)
      throw std::runtime_error("Selected calibration does not match the validated mount " + id);
    std::cout << (status == "validated" ? "Validated calibration" : "Calibration validation trial")
              << " matches mount " << id << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
