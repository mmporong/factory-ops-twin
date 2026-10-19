import type { Metadata } from "next";
import "./globals.css";
export const metadata: Metadata = {
  title: "Factory Ops · 생산 물류 관제",
  description: "4대 AMR와 생산 구역의 운영 상태",
};
export default function Layout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="ko">
      <body>{children}</body>
    </html>
  );
}
