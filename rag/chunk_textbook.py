#!/usr/bin/env python3
"""
RAG Content Chunking Script for Physical AI Textbook

This script processes markdown chapters and creates semantic chunks
suitable for vector embedding and retrieval-augmented generation.

Output: chunks.json with structured metadata
"""

import re
import json
from pathlib import Path
from typing import List, Dict, Tuple


class TextbookChunker:
    """Chunks textbook content into semantic segments for RAG systems."""

    def __init__(self, min_tokens=300, max_tokens=500):
        """
        Initialize chunker with token limits.

        Args:
            min_tokens: Minimum tokens per chunk
            max_tokens: Maximum tokens per chunk
        """
        self.min_tokens = min_tokens
        self.max_tokens = max_tokens

    def count_tokens(self, text: str) -> int:
        """
        Estimate token count using word-based approximation.

        Typical ratio: ~1.3 tokens per word for English text.
        This is a conservative estimate for GPT-style tokenization.
        """
        words = text.split()
        return int(len(words) * 1.3)

    def extract_metadata(self, filepath: Path) -> Dict:
        """Extract chapter metadata from filename and content."""
        filename = filepath.stem

        # Parse filename
        if filename.startswith('chapter-'):
            parts = filename.split('-', 2)
            chapter_num = int(parts[1])
            chapter_type = "chapter"
            title_slug = parts[2] if len(parts) > 2 else ""
        elif filename.startswith('appendix-'):
            parts = filename.split('-', 2)
            chapter_num = ord(parts[1]) - ord('a') + 1  # a=1, b=2, etc.
            chapter_type = "appendix"
            title_slug = parts[2] if len(parts) > 2 else ""
        else:
            chapter_num = 0
            chapter_type = "unknown"
            title_slug = filename

        return {
            "filename": filename,
            "chapter_type": chapter_type,
            "chapter_number": chapter_num,
            "title_slug": title_slug
        }

    def parse_markdown_structure(self, content: str) -> List[Dict]:
        """Parse markdown into structured sections."""
        lines = content.split('\n')
        sections = []
        current_section = {
            "level": 0,
            "title": "",
            "content": [],
            "path": []
        }

        heading_stack = []  # Track heading hierarchy

        for line in lines:
            # Check for headings
            heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)

            if heading_match:
                # Save previous section if it has content
                if current_section["content"]:
                    sections.append(current_section.copy())

                level = len(heading_match.group(1))
                title = heading_match.group(2).strip()

                # Update heading stack
                heading_stack = heading_stack[:level-1]  # Remove deeper levels
                if len(heading_stack) < level:
                    heading_stack.extend([''] * (level - len(heading_stack)))
                heading_stack[level-1] = title

                # Start new section
                current_section = {
                    "level": level,
                    "title": title,
                    "content": [],
                    "path": heading_stack[:level].copy()
                }
            else:
                # Add content to current section
                if line.strip():  # Skip empty lines for now, add back later
                    current_section["content"].append(line)

        # Don't forget the last section
        if current_section["content"]:
            sections.append(current_section)

        return sections

    def create_chunks(self, sections: List[Dict], metadata: Dict) -> List[Dict]:
        """Create semantic chunks from sections."""
        chunks = []
        chunk_id = 1

        for section in sections:
            section_text = '\n'.join(section["content"])
            section_tokens = self.count_tokens(section_text)

            # Small section - keep as single chunk
            if section_tokens <= self.max_tokens:
                chunk = self._create_chunk(
                    chunk_id=chunk_id,
                    content=section_text,
                    section=section,
                    metadata=metadata,
                    part_number=1,
                    total_parts=1
                )
                chunks.append(chunk)
                chunk_id += 1

            # Large section - split by paragraphs
            else:
                paragraphs = self._split_into_paragraphs(section["content"])
                sub_chunks = self._chunk_paragraphs(
                    paragraphs,
                    section,
                    metadata,
                    chunk_id
                )
                chunks.extend(sub_chunks)
                chunk_id += len(sub_chunks)

        return chunks

    def _split_into_paragraphs(self, lines: List[str]) -> List[str]:
        """Split content lines into paragraphs."""
        paragraphs = []
        current_para = []

        for line in lines:
            if line.strip():
                current_para.append(line)
            elif current_para:
                paragraphs.append('\n'.join(current_para))
                current_para = []

        if current_para:
            paragraphs.append('\n'.join(current_para))

        return paragraphs

    def _chunk_paragraphs(
        self,
        paragraphs: List[str],
        section: Dict,
        metadata: Dict,
        start_chunk_id: int
    ) -> List[Dict]:
        """Chunk paragraphs into token-limited segments."""
        chunks = []
        current_chunk_text = []
        current_tokens = 0
        part_number = 1

        for para in paragraphs:
            para_tokens = self.count_tokens(para)

            # Paragraph too large - split further if needed
            if para_tokens > self.max_tokens:
                # Save current chunk if any
                if current_chunk_text:
                    chunk_text = '\n\n'.join(current_chunk_text)
                    chunks.append(self._create_chunk(
                        chunk_id=start_chunk_id + len(chunks),
                        content=chunk_text,
                        section=section,
                        metadata=metadata,
                        part_number=part_number,
                        total_parts=-1  # Will update later
                    ))
                    current_chunk_text = []
                    current_tokens = 0
                    part_number += 1

                # Split large paragraph by sentences
                sentences = self._split_into_sentences(para)
                for sentence in sentences:
                    sent_tokens = self.count_tokens(sentence)

                    if current_tokens + sent_tokens > self.max_tokens:
                        if current_chunk_text:
                            chunk_text = '\n\n'.join(current_chunk_text)
                            chunks.append(self._create_chunk(
                                chunk_id=start_chunk_id + len(chunks),
                                content=chunk_text,
                                section=section,
                                metadata=metadata,
                                part_number=part_number,
                                total_parts=-1
                            ))
                            current_chunk_text = []
                            current_tokens = 0
                            part_number += 1

                    current_chunk_text.append(sentence)
                    current_tokens += sent_tokens

            # Normal paragraph handling
            elif current_tokens + para_tokens > self.max_tokens:
                # Save current chunk
                if current_chunk_text:
                    chunk_text = '\n\n'.join(current_chunk_text)
                    chunks.append(self._create_chunk(
                        chunk_id=start_chunk_id + len(chunks),
                        content=chunk_text,
                        section=section,
                        metadata=metadata,
                        part_number=part_number,
                        total_parts=-1
                    ))
                    current_chunk_text = []
                    current_tokens = 0
                    part_number += 1

                current_chunk_text.append(para)
                current_tokens = para_tokens

            else:
                current_chunk_text.append(para)
                current_tokens += para_tokens

        # Save final chunk
        if current_chunk_text:
            chunk_text = '\n\n'.join(current_chunk_text)
            chunks.append(self._create_chunk(
                chunk_id=start_chunk_id + len(chunks),
                content=chunk_text,
                section=section,
                metadata=metadata,
                part_number=part_number,
                total_parts=-1
            ))

        # Update total_parts for all chunks in this section
        total_parts = len(chunks)
        for chunk in chunks:
            chunk["total_parts"] = total_parts

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """Split text into sentences."""
        # Simple sentence splitting (can be improved)
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def _create_chunk(
        self,
        chunk_id: int,
        content: str,
        section: Dict,
        metadata: Dict,
        part_number: int,
        total_parts: int
    ) -> Dict:
        """Create a chunk with full metadata."""
        token_count = self.count_tokens(content)

        return {
            "chunk_id": f"{metadata['filename']}_chunk_{chunk_id:04d}",
            "content": content.strip(),
            "metadata": {
                "chapter_type": metadata["chapter_type"],
                "chapter_number": metadata["chapter_number"],
                "chapter_title_slug": metadata["title_slug"],
                "filename": metadata["filename"],
                "section_level": section["level"],
                "section_title": section["title"],
                "section_path": section["path"],
                "heading_hierarchy": " > ".join(section["path"]),
                "part_number": part_number,
                "total_parts": total_parts,
                "token_count": token_count,
                "char_count": len(content)
            }
        }

    def process_file(self, filepath: Path) -> List[Dict]:
        """Process a single markdown file into chunks."""
        print(f"Processing: {filepath.name}")

        # Extract metadata
        metadata = self.extract_metadata(filepath)

        # Read content
        content = filepath.read_text(encoding='utf-8')

        # Parse structure
        sections = self.parse_markdown_structure(content)

        # Create chunks
        chunks = self.create_chunks(sections, metadata)

        print(f"  Generated {len(chunks)} chunks")
        return chunks

    def process_directory(self, directory: Path) -> List[Dict]:
        """Process all markdown files in a directory."""
        all_chunks = []

        # Get all markdown files, sorted
        md_files = sorted(directory.glob("*.md"))

        print(f"\nProcessing {len(md_files)} files from {directory}\n")

        for md_file in md_files:
            chunks = self.process_file(md_file)
            all_chunks.extend(chunks)

        return all_chunks

    def save_chunks(self, chunks: List[Dict], output_file: Path):
        """Save chunks to JSON file."""
        output_data = {
            "metadata": {
                "source": "Physical AI & Humanoid Robotics Textbook",
                "total_chunks": len(chunks),
                "min_tokens": self.min_tokens,
                "max_tokens": self.max_tokens,
                "token_estimation": "word_based (1.3x words)"
            },
            "chunks": chunks
        }

        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)

        print(f"\n✅ Saved {len(chunks)} chunks to {output_file}")

        # Print statistics
        token_counts = [chunk["metadata"]["token_count"] for chunk in chunks]
        print(f"\n📊 Statistics:")
        print(f"   Total chunks: {len(chunks)}")
        print(f"   Avg tokens/chunk: {sum(token_counts) / len(token_counts):.1f}")
        print(f"   Min tokens: {min(token_counts)}")
        print(f"   Max tokens: {max(token_counts)}")


def main():
    """Main execution function."""
    # Setup paths
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    chapters_dir = project_root / "book" / "docs" / "chapters"
    output_file = script_dir / "chunks.json"

    print("=" * 60)
    print("Physical AI Textbook - RAG Content Chunker")
    print("=" * 60)

    # Initialize chunker
    chunker = TextbookChunker(min_tokens=300, max_tokens=500)

    # Process all chapters
    chunks = chunker.process_directory(chapters_dir)

    # Save output
    chunker.save_chunks(chunks, output_file)

    print("\n" + "=" * 60)
    print("✅ Processing complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
