"use client";
import { useEffect, useMemo, useRef, useState } from "react";
import Icon from "./Icon";
import type { SearchItem } from "../lib/ops-view";
import type { Selection } from "../lib/types";

export default function SearchBox({ items, onPick }: { items: SearchItem[]; onPick: (selection: Selection) => void }) {
  const [query, setQuery] = useState("");
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState(0);
  const input = useRef<HTMLInputElement>(null);
  const results = useMemo(() => {
    const words = query.trim().toLowerCase().split(/\s+/).filter(Boolean);
    if (!words.length) return items.filter((i) => i.group === "AMR" || i.group === "스테이션").slice(0, 8);
    return items.filter((i) => words.every((w) => i.terms.includes(w))).slice(0, 8);
  }, [items, query]);

  useEffect(() => {
    function onKey(e: KeyboardEvent) {
      const target = e.target as HTMLElement | null;
      const typing = target && (target.isContentEditable || ["INPUT", "SELECT", "TEXTAREA"].includes(target.tagName));
      if (e.key === "/" && !typing && !e.metaKey && !e.ctrlKey && !e.altKey) {
        e.preventDefault();
        input.current?.focus();
      }
    }
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  function pick(item: SearchItem | undefined) {
    if (!item) return;
    onPick(item.selection);
    setQuery("");
    setOpen(false);
    input.current?.blur();
  }

  const expanded = open && results.length > 0;
  return (
    <div className="search">
      <Icon name="search" size={17} />
      <input
        ref={input}
        type="search"
        role="combobox"
        aria-label="AMR·주문·설비·스테이션 검색"
        aria-expanded={expanded}
        aria-controls="search-results"
        aria-autocomplete="list"
        aria-activedescendant={expanded ? `search-option-${active}` : undefined}
        placeholder="AMR, 주문, lot, 설비 검색…"
        autoComplete="off"
        spellCheck={false}
        value={query}
        onChange={(e) => {
          setQuery(e.target.value);
          setActive(0);
          setOpen(true);
        }}
        onFocus={() => setOpen(true)}
        onBlur={() => setOpen(false)}
        onKeyDown={(e) => {
          if (e.key === "ArrowDown") {
            e.preventDefault();
            setOpen(true);
            setActive((i) => Math.min(results.length - 1, i + 1));
          } else if (e.key === "ArrowUp") {
            e.preventDefault();
            setActive((i) => Math.max(0, i - 1));
          } else if (e.key === "Enter") {
            e.preventDefault();
            pick(results[active]);
          } else if (e.key === "Escape") {
            if (open) setOpen(false);
            else setQuery("");
          }
        }}
      />
      <kbd aria-hidden="true">/</kbd>
      {expanded && (
        <ul id="search-results" role="listbox" aria-label="검색 결과" className="search-results">
          {results.map((item, i) => (
            <li
              key={item.key}
              id={`search-option-${i}`}
              role="option"
              aria-selected={i === active}
              className={i === active ? "is-active" : ""}
              onMouseDown={(e) => {
                e.preventDefault();
                pick(item);
              }}
              onMouseEnter={() => setActive(i)}
            >
              <span className="search-group">{item.group}</span>
              <strong>{item.title}</strong>
              <small>{item.subtitle}</small>
            </li>
          ))}
        </ul>
      )}
      {open && query.trim() && results.length === 0 && (
        <p className="search-empty" role="status">“{query.trim()}”와 일치하는 대상이 없습니다.</p>
      )}
    </div>
  );
}
