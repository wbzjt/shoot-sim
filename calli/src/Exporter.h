#pragma once

#include <string>

#include "Types.h"

namespace calibration {

class Exporter {
 public:
  bool ExportAll(const AnalysisOutput& output,
                 const CalibrationConfig& config,
                 const ExportPaths& paths,
                 std::string* error) const;

 private:
  bool ExportRecommendedPointsCsv(const AnalysisOutput& output,
                                  const std::string& path,
                                  std::string* error) const;
  bool ExportAllScoresCsv(const AnalysisOutput& output,
                          const std::string& path,
                          std::string* error) const;
  bool ExportPlannerSummaryJson(const AnalysisOutput& output,
                                const std::string& path,
                                std::string* error) const;
  bool ExportAggregatedCsv(const AnalysisOutput& output,
                           const std::string& path,
                           std::string* error) const;
  bool ExportModelReportCsv(const AnalysisOutput& output,
                            const std::string& path,
                            std::string* error) const;
  bool ExportFitCurveCsv(const AnalysisOutput& output,
                         const std::string& path,
                         std::string* error) const;
  bool ExportFitResidualCsv(const AnalysisOutput& output,
                            const std::string& path,
                            std::string* error) const;
  bool ExportJson(const AnalysisOutput& output,
                  const CalibrationConfig& config,
                  const std::string& path,
                  std::string* error) const;
  bool ExportMarkdown(const AnalysisOutput& output,
                      const std::string& path,
                      std::string* error) const;
  bool ExportHeader(const AnalysisOutput& output,
                    const std::string& path,
                    std::string* error) const;
};

}  // namespace calibration
