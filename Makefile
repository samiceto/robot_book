# Makefile for Physical AI & Humanoid Robotics Book
# Converts Markdown to PDF using Pandoc

# Variables
PANDOC := pandoc
BOOK_DIR := book
CHAPTERS_DIR := $(BOOK_DIR)/chapters
OUTPUT_DIR := build
TEMPLATE := .pandoc/template.latex
BIBLIOGRAPHY := references.bib

# Chapter files (in order)
PART1 := $(CHAPTERS_DIR)/part1-foundations/*.md
PART2 := $(CHAPTERS_DIR)/part2-simulation/*.md
PART3 := $(CHAPTERS_DIR)/part3-perception-edge/*.md
PART4 := $(CHAPTERS_DIR)/part4-embodied-cognition/*.md
PART5 := $(CHAPTERS_DIR)/part5-locomotion/*.md
PART6 := $(CHAPTERS_DIR)/part6-capstone/*.md
APPENDICES := $(BOOK_DIR)/appendices/*.md
FRONTMATTER := $(BOOK_DIR)/frontmatter/*.md

ALL_CHAPTERS := $(wildcard $(PART1) $(PART2) $(PART3) $(PART4) $(PART5) $(PART6))

# Output files
PDF_OUTPUT := $(OUTPUT_DIR)/physical-ai-humanoid-robotics.pdf
EPUB_OUTPUT := $(OUTPUT_DIR)/physical-ai-humanoid-robotics.epub

# Pandoc flags
PANDOC_FLAGS := --template=$(TEMPLATE) \
                --toc \
                --number-sections \
                --listings \
                --pdf-engine=xelatex \
                -V geometry:margin=1in \
                -V linkcolor:blue \
                -V urlcolor:blue

.PHONY: all pdf epub clean

all: pdf

pdf: $(PDF_OUTPUT)

epub: $(EPUB_OUTPUT)

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

$(PDF_OUTPUT): $(ALL_CHAPTERS) | $(OUTPUT_DIR)
	$(PANDOC) $(PANDOC_FLAGS) \
		$(FRONTMATTER) \
		$(ALL_CHAPTERS) \
		$(APPENDICES) \
		-o $@

$(EPUB_OUTPUT): $(ALL_CHAPTERS) | $(OUTPUT_DIR)
	$(PANDOC) --toc \
		--epub-cover-image=book/images/cover.png \
		$(FRONTMATTER) \
		$(ALL_CHAPTERS) \
		$(APPENDICES) \
		-o $@

clean:
	rm -rf $(OUTPUT_DIR)

# Individual chapter builds for testing
chapter-%: $(CHAPTERS_DIR)/%/*.md | $(OUTPUT_DIR)
	$(PANDOC) $(PANDOC_FLAGS) $< -o $(OUTPUT_DIR)/chapter-$*.pdf
