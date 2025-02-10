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
  std::size_t mem_delta_compressed_bytes = 0;
  std::size_t mem_simple_compressed_bytes = 0;

  std::array<std::vector<Label>, 2> labels;

  readFromFile(labels, inputFileName);

  showStats(labels);

  /* runQueries(labels); */

  mem_raw_bytes = computeTotalBytes(labels);

  {
    std::array<std::vector<CompressedLabel>, 2> comp_labels;

    comp_labels[FWD].reserve(labels[FWD].size());
    comp_labels[BWD].reserve(labels[BWD].size());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      comp_labels[FWD].emplace_back(labels[FWD][v]);
      comp_labels[BWD].emplace_back(labels[BWD][v]);
    }

    for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
      mem_compressed_bytes += comp_labels[FWD][v].byteSize();
      mem_compressed_bytes += comp_labels[BWD][v].byteSize();
    }
  }
  {
    std::array<std::vector<DeltaCompressedLabel>, 2> comp_labels;

    comp_labels[FWD].reserve(labels[FWD].size());
    comp_labels[BWD].reserve(labels[BWD].size());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      comp_labels[FWD].emplace_back(labels[FWD][v]);
      comp_labels[BWD].emplace_back(labels[BWD][v]);
    }

    for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
      mem_delta_compressed_bytes += comp_labels[FWD][v].byteSize();
      mem_delta_compressed_bytes += comp_labels[BWD][v].byteSize();
    }
  }

  {
    std::array<std::vector<SimpleCompressedLabel>, 2> comp_labels;

    comp_labels[FWD].reserve(labels[FWD].size());
    comp_labels[BWD].reserve(labels[BWD].size());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      comp_labels[FWD].emplace_back(labels[FWD][v]);
      comp_labels[BWD].emplace_back(labels[BWD][v]);
    }

    for (std::size_t v = 0; v < comp_labels[FWD].size(); ++v) {
      mem_simple_compressed_bytes += comp_labels[FWD][v].byteSize();
      mem_simple_compressed_bytes += comp_labels[BWD][v].byteSize();
    }
  }
  std::cout << "Memory [mb]:        " << std::endl;
  std::cout << "Raw:                " << (mem_raw_bytes / (1024 * 1024))
            << std::endl;
  std::cout << "Compressed:         " << (mem_compressed_bytes / (1024 * 1024))
            << std::endl;
  std::cout << "Delta Compressed:   "
            << (mem_delta_compressed_bytes / (1024 * 1024)) << std::endl;
  std::cout << "Simple Compressed:  "
            << (mem_simple_compressed_bytes / (1024 * 1024)) << std::endl;

  return 0;
}

/*
# this is without -c (compress)
./LabelVSCompLabel -i ../data/icice.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     222
  Avg Size:     64.8244
Backward Labels Statistics:
  Min Size:     1
  Max Size:     133
  Avg Size:     42.7774
FWD # count:    12061749
BWD # count:    7959504
Both # count:   20021253
Total memory consumption [megabytes]:
  117.453
Memory [mb]:
Raw:                117
Compressed:         78
Simple Compressed:  125

./LabelVSCompLabel -i ../data/kvv.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.5587
Backward Labels Statistics:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6286
FWD # count:    198548559
BWD # count:    175978560
Both # count:   374527119
Total memory consumption [megabytes]:
  2149.56
Memory [mb]:
Raw:                2149
Compressed:         1431
Simple Compressed:  2265

# this is with -c (compress)
Forward Labels Statistics:
  Min Size:     1
  Max Size:     222
  Avg Size:     64.8377
Backward Labels Statistics:
  Min Size:     1
  Max Size:     133
  Avg Size:     42.776
FWD # count:    12064218
BWD # count:    7959240
Both # count:   20023458
Total memory consumption [megabytes]:
  117.455
Memory [mb]:
Raw:                117
Compressed:         58
Simple Compressed:  103

./LabelVSCompLabel -i ../data/kvv.labels
Forward Labels Statistics:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.5586
Backward Labels Statistics:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6287
FWD # count:    198548139
BWD # count:    175978837
Both # count:   374526976
Total memory consumption [megabytes]:
  2149.57
Memory [mb]:
Raw:                2149
Compressed:         1130
Simple Compressed:  2089
*/
