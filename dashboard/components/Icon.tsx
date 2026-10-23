const paths = {
  search: "M11 4a7 7 0 1 0 0 14 7 7 0 0 0 0-14Zm9 16-4.2-4.2",
  plus: "M12 5v14M5 12h14",
  minus: "M5 12h14",
  rotateLeft: "M4 4v5h5M4.6 9A8 8 0 1 1 6 17.6",
  rotateRight: "M20 4v5h-5M19.4 9A8 8 0 1 0 18 17.6",
  home: "M4 11 12 4l8 7M6 9.5V20h12V9.5",
  focus: "M12 3v3M12 18v3M3 12h3M18 12h3M12 8a4 4 0 1 0 0 8 4 4 0 0 0 0-8Z",
  layers: "m12 4 9 5-9 5-9-5 9-5Zm-9 9 9 5 9-5",
  close: "M6 6l12 12M18 6 6 18",
  chevronRight: "m9 6 6 6-6 6",
  chevronLeft: "m15 6-6 6 6 6",
  chevronDown: "m6 9 6 6 6-6",
  sliders: "M5 6h9M18 6h1M5 12h3M12 12h7M5 18h11M20 18h-1M14 4v4M8 10v4M16 16v4",
  pause: "M8 5v14M16 5v14",
  play: "M7 5v14l11-7L7 5Z",
  robot: "M5 10h14v7H5zM8 17v2M16 17v2M9 7h6v3H9zM12 4v3",
  box: "m12 3 8 4.5v9L12 21l-8-4.5v-9L12 3Zm0 9 8-4.5M12 12 4 7.5M12 12v9",
  check: "m5 12.5 4.5 4.5L19 7",
  truck: "M3 7h11v9H3zM14 10h4l3 3v3h-7M7 19a1.5 1.5 0 1 0 0-3 1.5 1.5 0 0 0 0 3Zm10 0a1.5 1.5 0 1 0 0-3 1.5 1.5 0 0 0 0 3Z",
  factory: "M3 20V9l5 3V9l5 3V5h4l1 15M3 20h18",
  bolt: "M13 3 5 13h6l-1 8 8-10h-6l1-8Z",
  list: "M8 6h12M8 12h12M8 18h12M4 6h.01M4 12h.01M4 18h.01",
  record: "M12 7a5 5 0 1 0 0 10 5 5 0 0 0 0-10Z",
  shield: "M12 3 5 6v6c0 4 3 7.5 7 9 4-1.5 7-5 7-9V6l-7-3Z",
  pin: "M12 21s7-6.2 7-11a7 7 0 1 0-14 0c0 4.8 7 11 7 11Zm0-9a2 2 0 1 0 0-4 2 2 0 0 0 0 4Z",
  stack: "M4 7h16v4H4zM4 13h16v4H4z",
  gauge: "M4 15a8 8 0 1 1 16 0M12 15l4-5",
  route: "M6 19a2 2 0 1 0 0-4 2 2 0 0 0 0 4Zm12-10a2 2 0 1 0 0-4 2 2 0 0 0 0 4ZM8 17h7a3 3 0 0 0 0-6H9a3 3 0 0 1 0-6h7",
};

export type IconName = keyof typeof paths;

export default function Icon({ name, size = 18 }: { name: IconName; size?: number }) {
  return (
    <svg
      className="icon"
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.9}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
      focusable="false"
    >
      <path d={paths[name]} />
    </svg>
  );
}
