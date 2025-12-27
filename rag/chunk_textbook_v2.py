#!/usr/bin/env python3
"""
RAG Content Chunking Script v2 - Optimized for 300-500 tokens

Improvements:
- Better merging of small sections
- Smarter chunk boundaries
- More consistent token counts
"""

import re
import json
from pathlib import Path
from typing import List, Dict


class OptimizedTextbookChunker:
    """Optimized chunker for RAG-ready semantic segments."""

    def __init__(self, target_min=300, target_max=500):
        self.target_min = target_min
        self.target_max = target_max

    def count_tokens(self, text: str) -> int:
        """Estimate tokens (1.3x words for English)."""
        return int(len(text.split()) * 1.3)

    def extract_metadata(self, filepath: Path) -> Dict:
        """Extract chapter metadata."""
        filename = filepath.stem

        if filename.startswith('chapter-'):
            parts = filename.split('-', 2)
            chapter_num = int(parts[1])
            chapter_type = "chapter"
            title_slug = parts[2] if len(parts) > 2 else ""
        elif filename.startswith('appendix-'):
            parts = filename.split('-', 2)
            chapter_num = ord(parts[1]) - ord('a') + 1
            chapter_type = "appendix"
            title_slug = parts[2] if len(parts) > 2 else ""
        else:
            chapter_num = 0
            chapter_type = "other"
            title_slug = filename

        return {
            "filename": filename,
            "chapter_type": chapter_type,
            "chapter_number": chapter_num,
            "title_slug": title_slug
        }

    def parse_content(self, content: str) -> List[Dict]:
        """Parse markdown into sections with full content."""
        lines = content.split('\n')
        sections = []
        current_section = None
        heading_stack = []

        for line in lines:
            heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)

            if heading_match:
                # Save previous section
                if current_section and current_section["lines"]:
                    sections.append(current_section)

                level = len(heading_match.group(1))
                title = heading_match.group(2).strip()

                # Update heading hierarchy
                heading_stack = heading_stack[:level-1]
                if len(heading_stack) < level:
                    heading_stack.extend([''] * (level - len(heading_stack)))
                heading_stack[level-1] = title

                # New section
                current_section = {
                    "level": level,
                    "title": title,
                    "path": heading_stack[:level].copy(),
                    "lines": []
                }
            elif current_section is not None:
                current_section["lines"].append(line)

        # Don't forget last section
        if current_section and current_section["lines"]:
            sections.append(current_section)

        return sections

    def merge_small_sections(self, sections: List[Dict]) -> List[Dict]:
        """Merge consecutive small sections to meet token targets."""
        if not sections:
            return []

        merged = []
        accumulator = None

        for section in sections:
            section_text = '\n'.join(section["lines"]).strip()
            section_tokens = self.count_tokens(section_text)

            if accumulator is None:
                # Start new accumulator
                accumulator = {
                    "content": section_text,
                    "tokens": section_tokens,
                    "sections": [section]
                }
            elif (accumulator["tokens"] + section_tokens < self.target_max and
                  accumulator["sections"][-1]["level"] >= section["level"]):
                # Merge into accumulator (same or deeper level)
                accumulator["content"] += "\n\n" + section_text
                accumulator["tokens"] += section_tokens
                accumulator["sections"].append(section)
            else:
                # Save accumulator and start new one
                merged.append(accumulator)
                accumulator = {
                    "content": section_text,
                    "tokens": section_tokens,
                    "sections": [section]
                }

        # Don't forget last accumulator
        if accumulator:
            merged.append(accumulator)

        return merged

    def split_large_chunk(self, content: str, metadata: Dict) -> List[str]:
        """Split large content into smaller chunks at paragraph boundaries."""
        paragraphs = re.split(r'\n\s*\n', content)
        chunks = []
        current = []
        current_tokens = 0

        for para in paragraphs:
            para = para.strip()
            if not para:
                continue

            para_tokens = self.count_tokens(para)

            # Paragraph itself too large - keep as single chunk if needed
            if para_tokens > self.target_max:
                if current:
                    chunks.append('\n\n'.join(current))
                    current = []
                    current_tokens = 0
                chunks.append(para)
                continue

            # Would exceed target - save current and start new
            if current_tokens + para_tokens > self.target_max:
                if current:
                    chunks.append('\n\n'.join(current))
                current = [para]
                current_tokens = para_tokens
            else:
                current.append(para)
                current_tokens += para_tokens

        # Save final chunk
        if current:
            chunks.append('\n\n'.join(current))

        return chunks

    def create_chunks(self, merged_sections: List[Dict], file_metadata: Dict) -> List[Dict]:
        """Create final chunks from merged sections."""
        chunks = []
        chunk_id = 1

        for merged in merged_sections:
            content = merged["content"]
            tokens = merged["tokens"]
            sections = merged["sections"]

            # Get primary section (first one)
            primary_section = sections[0]

            # Content fits in one chunk
            if tokens <= self.target_max:
                chunk = self._make_chunk(
                    chunk_id=chunk_id,
                    content=content,
                    section=primary_section,
                    file_metadata=file_metadata,
                    part=1,
                    total=1
                )
                chunks.append(chunk)
                chunk_id += 1

            # Need to split large content
            else:
                split_content = self.split_large_chunk(content, file_metadata)
                for i, chunk_text in enumerate(split_content, 1):
                    chunk = self._make_chunk(
                        chunk_id=chunk_id,
                        content=chunk_text,
                        section=primary_section,
                        file_metadata=file_metadata,
                        part=i,
                        total=len(split_content)
                    )
                    chunks.append(chunk)
                    chunk_id += 1

        return chunks

    def _make_chunk(
        self,
        chunk_id: int,
        content: str,
        section: Dict,
        file_metadata: Dict,
        part: int,
        total: int
    ) -> Dict:
        """Create chunk with metadata."""
        return {
            "chunk_id": f"{file_metadata['filename']}_chunk_{chunk_id:04d}",
            "content": content.strip(),
            "metadata": {
                "chapter_type": file_metadata["chapter_type"],
                "chapter_number": file_metadata["chapter_number"],
                "chapter_title_slug": file_metadata["title_slug"],
                "filename": file_metadata["filename"],
                "section_level": section["level"],
                "section_title": section["title"],
                "section_path": section["path"],
                "heading_hierarchy": " > ".join(section["path"]),
                "part_number": part,
                "total_parts": total,
                "token_count": self.count_tokens(content),
                "char_count": len(content)
            }
        }

    def process_file(self, filepath: Path) -> List[Dict]:
        """Process single markdown file."""
        print(f"Processing: {filepath.name}")

        file_metadata = self.extract_metadata(filepath)
        content = filepath.read_text(encoding='utf-8')

        sections = self.parse_content(content)
        merged = self.merge_small_sections(sections)
        chunks = self.create_chunks(merged, file_metadata)

        print(f"  → {len(chunks)} chunks")
        return chunks

    def process_directory(self, directory: Path) -> List[Dict]:
        """Process all markdown files."""
        all_chunks = []
        md_files = sorted(directory.glob("*.md"))

        print(f"\n{'='*60}")
        print(f"Processing {len(md_files)} files")
        print(f"{'='*60}\n")

        for md_file in md_files:
            chunks = self.process_file(md_file)
            all_chunks.extend(chunks)

        return all_chunks

    def save_chunks(self, chunks: List[Dict], output_file: Path):
        """Save chunks with statistics."""
        output_data = {
            "metadata": {
                "source": "Physical AI & Humanoid Robotics Textbook",
                "description": "Semantic chunks optimized for RAG retrieval",
                "total_chunks": len(chunks),
                "target_tokens_min": self.target_min,
                "target_tokens_max": self.target_max,
                "token_estimation_method": "word_based (1.3x words)"
            },
            "chunks": chunks
        }

        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)

        # Statistics
        tokens = [c["metadata"]["token_count"] for c in chunks]
        in_range = sum(1 for t in tokens if self.target_min <= t <= self.target_max)

        print(f"\n{'='*60}")
        print(f"✅ Saved to: {output_file}")
        print(f"{'='*60}\n")
        print(f"📊 Statistics:")
        print(f"   Total chunks: {len(chunks)}")
        print(f"   Avg tokens: {sum(tokens) / len(tokens):.1f}")
        print(f"   Min tokens: {min(tokens)}")
        print(f"   Max tokens: {max(tokens)}")
        print(f"   In target range ({self.target_min}-{self.target_max}): {in_range} ({in_range/len(chunks)*100:.1f}%)")
        print(f"\n{'='*60}")


def main():
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    chapters_dir = project_root / "book" / "docs" / "chapters"
    output_file = script_dir / "chunks.json"

    print("\n" + "="*60)
    print("Physical AI Textbook - RAG Content Chunker v2")
    print("="*60)

    chunker = OptimizedTextbookChunker(target_min=300, target_max=500)
    chunks = chunker.process_directory(chapters_dir)
    chunker.save_chunks(chunks, output_file)


if __name__ == "__main__":
    main()
