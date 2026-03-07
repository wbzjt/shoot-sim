#include "CalibrationApp.h"

#include <filesystem>
#include <iostream>
#include <map>
#include <string>

#include "CalibrationAggregator.h"
#include "CalibrationDataLoader.h"
#include "CalibrationPlanner.h"
#include "Exporter.h"
#include "MathUtil.h"
#include "ModelFitter.h"
#include "SegmentAnalyzer.h"
#include "TheoryDataLoader.h"

namespace calibration {
namespace {

std::map<std::string, std::string> ParseArgs(int argc, char** argv, int start) {
  std::map<std::string, std::string> out;
  for (int i = start; i < argc; ++i) {
    const std::string token = argv[i];
    if (token.rfind("--", 0) == 0) {
      if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        out[token] = argv[i + 1];
        ++i;
      } else {
        out[token] = "1";
      }
    }
  }
  return out;
}

std::string GetOrDefault(const std::map<std::string, std::string>& args,
                         const std::string& key,
                         const std::string& fallback) {
  const auto it = args.find(key);
  if (it == args.end()) {
    return fallback;
  }
  return it->second;
}

void PrintUsage() {
  std::cout << "ShooterCalibrationTool usage:\n"
            << "  ShooterCalibrationTool full --theory <theory.csv> --calib <calibration.csv> --out-dir <dir> [--target-count N] [--mode uniform|sensitivity]\n"
            << "  ShooterCalibrationTool plan --theory <theory.csv> --out-dir <dir> [--target-count N] [--mode uniform|sensitivity]\n"
            << "  ShooterCalibrationTool plan --input <theory.csv> --output <recommended_points.csv> [--target-count N] [--error-threshold E] [--use-delta-v true|false] [--verbose]\n"
            << "  ShooterCalibrationTool fit  --theory <theory.csv> --calib <calibration.csv> --out-dir <dir>\n";
}

bool BuildAnalysis(const std::string& theoryCsv,
                   const std::string& calibCsv,
                   const CalibrationConfig& config,
                   AnalysisOutput* out,
                   std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) {
      *error = "Analysis output is null";
    }
    return false;
  }

  TheoryDataLoader theoryLoader;
  if (!theoryLoader.LoadFromCsv(theoryCsv, &out->theoryPoints, error)) {
    return false;
  }

  CalibrationPlanner planner;
  std::string plannerWarn;
  PlannerSummary plannerSummary;
  out->recommendedPoints = planner.Recommend(
      out->theoryPoints, config.planner, &plannerSummary, &out->plannerAllScores, &plannerWarn);
  out->plannerSummary = plannerSummary;

  CalibrationDataLoader calibLoader;
  if (!calibLoader.LoadFromCsv(calibCsv, &out->rawSamples, error)) {
    return false;
  }

  CalibrationAggregator aggregator;
  std::string aggWarn;
  out->aggregatedPoints = aggregator.Aggregate(out->rawSamples, config.aggregation, &aggWarn);

  ModelFitter fitter;
  std::string fitWarn;
  out->modelReports = fitter.Fit(out->aggregatedPoints, config.fitter, &fitWarn);
  out->bestSafeModel = ModelFitter::PickBestSafe(out->modelReports);
  out->bestAccuracyModel = ModelFitter::PickBestAccuracy(out->modelReports);

  // Runtime recommendation: prefer best safe model.
  out->recommendedRuntimeModel = out->bestSafeModel;

  SegmentAnalyzer segmentAnalyzer;
  out->segmentRecommendation = segmentAnalyzer.Analyze(
      ModelFitter::BuildFitDataset(out->aggregatedPoints),
      out->modelReports,
      config.segment);

  if (!plannerWarn.empty()) {
    std::cout << "[planner] " << plannerWarn << "\n";
  }
  if (!aggWarn.empty()) {
    std::cout << "[aggregator] " << aggWarn << "\n";
  }
  if (!fitWarn.empty()) {
    std::cout << "[fitter] " << fitWarn << "\n";
  }

  return true;
}

}  // namespace

int CalibrationApp::Run(int argc, char** argv) const {
  if (argc < 2) {
    PrintUsage();
    return 1;
  }

  const std::string command = argv[1];
  const auto args = ParseArgs(argc, argv, 2);

  CalibrationConfig config;
  if (args.find("--target-count") != args.end()) {
    config.planner.recommendedCount = static_cast<std::size_t>(std::stoul(args.at("--target-count")));
  } else if (args.find("--recommend-count") != args.end()) {
    config.planner.recommendedCount = static_cast<std::size_t>(std::stoul(args.at("--recommend-count")));
  }
  if (args.find("--mode") != args.end()) {
    const std::string mode = math::ToLower(args.at("--mode"));
    config.planner.mode = (mode == "uniform") ? PlannerMode::kUniformCoverage
                                               : PlannerMode::kSensitivityFirst;
  }
  if (args.find("--error-threshold") != args.end()) {
    config.planner.maxInterpErrorThreshold = std::stod(args.at("--error-threshold"));
  }
  if (args.find("--use-delta-v") != args.end()) {
    bool flag = true;
    if (math::ParseBoolLoose(args.at("--use-delta-v"), &flag)) {
      config.planner.useDeltaV = flag;
    }
  }
  if (args.find("--verbose") != args.end()) {
    config.planner.verbose = true;
  }

  const std::string outDir = GetOrDefault(args, "--out-dir", "calibration_output");
  ExportPaths paths;
  paths.outDir = outDir;

  if (command == "plan") {
    const std::string theoryCsv = GetOrDefault(args, "--theory",
                                GetOrDefault(args, "--input", ""));
    if (theoryCsv.empty()) {
      std::cerr << "Missing --theory / --input\n";
      return 2;
    }

    if (args.find("--output") != args.end()) {
      const std::filesystem::path outFile = args.at("--output");
      const auto parent = outFile.parent_path();
      paths.outDir = parent.empty() ? "." : parent.string();
      paths.recommendedPointsCsv = outFile.filename().string();
      const std::string stem = outFile.stem().string();
      paths.plannerSummaryJson = stem + "_summary.json";
      paths.analysisJson = stem + "_analysis.json";
      paths.markdownReport = stem + "_summary.md";
      paths.allScoresCsv = stem + "_all_scores.csv";
    }

    AnalysisOutput out;
    TheoryDataLoader loader;
    std::string err;
    if (!loader.LoadFromCsv(theoryCsv, &out.theoryPoints, &err)) {
      std::cerr << err << "\n";
      return 3;
    }

    CalibrationPlanner planner;
    std::string warn;
    PlannerSummary plannerSummary;
    out.recommendedPoints = planner.Recommend(
        out.theoryPoints, config.planner, &plannerSummary, &out.plannerAllScores, &warn);
    out.plannerSummary = plannerSummary;

    try {
      std::filesystem::create_directories(paths.outDir);
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << "\n";
      return 4;
    }

    Exporter exporter;
    if (!exporter.ExportAll(out, config, paths, &err)) {
      std::cerr << err << "\n";
      return 5;
    }

    if (out.plannerSummary.has_value()) {
      std::cout << "[planner] stop reason: " << out.plannerSummary->stopReasonText << "\n";
      std::cout << "[planner] selected " << out.plannerSummary->selectedPointCount
                << " / input " << out.plannerSummary->inputPointCount << "\n";
      std::cout << "[planner] max interp error: " << out.plannerSummary->maxInterpolationError << "\n";
    }
    std::cout << "Plan export completed: " << std::filesystem::absolute(paths.outDir).string() << "\n";
    return 0;
  }

  if (command == "fit" || command == "full") {
    const auto itTheory = args.find("--theory");
    const auto itCalib = args.find("--calib");
    if (itTheory == args.end() || itCalib == args.end()) {
      std::cerr << "Missing --theory or --calib\n";
      return 2;
    }

    AnalysisOutput out;
    std::string err;
    if (!BuildAnalysis(itTheory->second, itCalib->second, config, &out, &err)) {
      std::cerr << err << "\n";
      return 3;
    }

    Exporter exporter;
    if (!exporter.ExportAll(out, config, paths, &err)) {
      std::cerr << err << "\n";
      return 4;
    }

    std::cout << "Analysis export completed: " << std::filesystem::absolute(paths.outDir).string() << "\n";
    if (out.recommendedRuntimeModel.has_value()) {
      std::cout << "Recommended runtime model: "
                << static_cast<int>(out.recommendedRuntimeModel->kind) << "\n";
    }
    return 0;
  }

  PrintUsage();
  return 1;
}

}  // namespace calibration
