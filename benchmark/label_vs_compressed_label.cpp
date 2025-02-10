#include <array>
#include <vector>

#include "../datastructures/hub_labels.h"
#include "../external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_labels", "Input label file.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");

  std::size_t mem_raw_bytes = 0;
  std::size_t mem_compressed_bytes = 0;

  std::array<std::vector<Label>, 2> labels;

  readFromFile(labels, inputFileName);

  showStats(labels);

  /* runQueries(labels); */

  mem_raw_bytes = computeTotalBytes(labels);

  std::array<std::vector<CompressedLabel>, 2> comp_labels;

  comp_labels[FWD].reserve(labels[FWD].size());
  comp_labels[BWD].reserve(labels[BWD].size());

  for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
    comp_labels[FWD].emplace_back(labels[FWD][v]);
    comp_labels[BWD].emplace_back(labels[BWD][v]);
  }

  for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
    mem_compressed_bytes += comp_labels[FWD][v].nodes.byteSize();
    mem_compressed_bytes += comp_labels[BWD][v].nodes.byteSize();
  }

  std::cout << "Memory [mb]: " << std::endl;
  std::cout << "Raw:         " << (mem_raw_bytes / (1024 * 1024)) << std::endl;
  std::cout << "Compressed:  " << (mem_compressed_bytes / (1024 * 1024))
            << std::endl;

  return 0;
}

/*
./LabelVSCompLabel -i ../data/icice.labels 
Forward Labels Statistics:
  Min Size:     1
  Max Size:     222
  Avg Size:     64.7671
Backward Labels Statistics:
  Min Size:     1
  Max Size:     134
  Avg Size:     42.7313
FWD # count:    12051083
BWD # count:    7950923
Both # count:   20002006
Total memory consumption [megabytes]:
  117.277
Memory [mb]: 
Raw:         117
Compressed:  67

./LabelVSCompLabel -i ../data/kvv.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.5585
Backward Labels Statistics:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6286
FWD # count:    198548051
BWD # count:    175978603
Both # count:   374526654
Total memory consumption [megabytes]:
  2149.57
Memory [mb]:
Raw:         2149
Compressed:  1277
*/
