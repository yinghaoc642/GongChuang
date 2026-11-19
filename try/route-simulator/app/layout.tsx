import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "麦轮小车两轮路线动态仿真",
  description:
    "严格按 tmcode1.cpp 坐标渲染的 2400×2400 mm 场地、长方体车体、机械臂圆心和两轮国赛路线。",
  openGraph: {
    title: "麦轮小车两轮路线动态仿真",
    description: "TMCode 1 · 230×300 mm · 机械臂偏移 225 mm",
    images: ["/route-preview.png"],
  },
  icons: {
    icon: "/favicon.svg",
    shortcut: "/favicon.svg",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="zh-CN">
      <body
        className={`${geistSans.variable} ${geistMono.variable} antialiased`}
      >
        {children}
      </body>
    </html>
  );
}
