import * as React from "react"
import type { HeadFC, PageProps } from "gatsby"
import "../style/main.css"
import {NT4Provider} from "@frc-web-components/react";
import NoteSelector from "../components/NoteSelector";

const IndexPage: React.FC<PageProps> = () => (
  <NT4Provider address="localhost">
      <NoteSelector/>
  </NT4Provider>
)

export default IndexPage

export const Head: HeadFC = () => <title>Note Selector</title>
