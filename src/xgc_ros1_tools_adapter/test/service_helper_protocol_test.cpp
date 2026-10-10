#include "xgc_ros1_tools_adapter/service_helper_protocol.hpp"

#include <gtest/gtest.h>

#include <string>

namespace xgc_ros1_tools_adapter {
namespace {

TEST(ServiceHelperProtocol, ValidatesTheTwoPhaseCommitSequence) {
  constexpr std::uint64_t request_id = 41u;
  const ServiceHelperFrame request =
      makeServiceHelperRequest(request_id, 18u, kServiceMd5Bytes, 1024u, 2000u);
  EXPECT_TRUE(validateServiceHelperRequest(request));

  const ServiceHelperFrame ready =
      makeServiceHelperReady(request_id, ServiceHelperReadyStatus::kReady);
  EXPECT_TRUE(validateServiceHelperReady(ready, request_id));

  const ServiceHelperFrame commit = makeServiceHelperCommit(request_id);
  EXPECT_TRUE(validateServiceHelperCommit(commit, request_id));

  const ServiceHelperFrame response = makeServiceHelperResponse(
      request_id, ServiceHelperResponseStatus::kCompleted, 512u);
  EXPECT_TRUE(validateServiceHelperResponse(response, request_id));
}

TEST(ServiceHelperProtocol, RejectsMismatchedOrOversizedFrames) {
  ServiceHelperFrame request = makeServiceHelperRequest(
      1u, kMaximumServiceNameBytes + 1u, kServiceMd5Bytes, 0u, 0u);
  EXPECT_FALSE(validateServiceHelperRequest(request));

  request = makeServiceHelperRequest(1u, 1u, kServiceMd5Bytes,
                                     kMaximumServicePayloadBytes + 1u, 0u);
  EXPECT_FALSE(validateServiceHelperRequest(request));

  request = makeServiceHelperRequest(1u, 1u, kServiceMd5Bytes, 0u,
                                     kMaximumServiceWaitMilliseconds + 1u);
  EXPECT_FALSE(validateServiceHelperRequest(request));

  ServiceHelperFrame ready =
      makeServiceHelperReady(2u, ServiceHelperReadyStatus::kReady);
  EXPECT_FALSE(validateServiceHelperReady(ready, 1u));

  ServiceHelperFrame commit = makeServiceHelperCommit(1u);
  commit.reserved[0] = 1u;
  EXPECT_FALSE(validateServiceHelperCommit(commit, 1u));

  ServiceHelperFrame failed = makeServiceHelperResponse(
      1u, ServiceHelperResponseStatus::kCallFailed, 1u);
  EXPECT_FALSE(validateServiceHelperResponse(failed, 1u));
}

TEST(ServiceHelperProtocol, AcceptsOnlyCanonicalServiceMd5Digests) {
  EXPECT_TRUE(validServiceMd5("0123456789abcdef0123456789ABCDEF"));
  EXPECT_FALSE(validServiceMd5("0123456789abcdef"));
  EXPECT_FALSE(validServiceMd5("0123456789abcdef0123456789abcdeg"));
}

}  // namespace
}  // namespace xgc_ros1_tools_adapter
