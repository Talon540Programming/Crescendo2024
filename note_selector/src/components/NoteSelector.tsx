// @ts-ignore
import field from "../images/FieldLayout.png";
import * as React from "react";
import Note from "./Note";
import {useEntry} from "@frc-web-components/react";
import {useEffect, useState} from "react";


const NoteSelector = () => {
    // frc-web-components has a bug, this is the workaround
    // const [selectedNotes, setSelectedNotes] = useEntry<string[]>("/AutoManager/SelectedNotes", []);
    const [selectedNotesNTString, setSelectedNotesNT] = useEntry<string>("/AutoManager/SelectedNotes", "");
    const [selectedNotes, setSelectedNotes] = useState<string[]>([]);

    useEffect(() => {
        setSelectedNotes(selectedNotesNTString.split(","))
    }, []);

    useEffect(() => {
        setSelectedNotesNT(selectedNotes.toString());
    }, [selectedNotes]);

    const updateNote = (name: string, isSelected: boolean) => {
        if(isSelected && !selectedNotes.includes(name)) {
            setSelectedNotes(selectedNotes.concat(name));
        } else {
            setSelectedNotes(selectedNotes.filter(v => v != name));
        }
    }

    return (
        <div className="field-wrapper">
            <img className="field-image" src={field} alt="field"/>
            <Note onChange={(value) => updateNote("Left One", value)} posX={357} posY={272} isSelected={selectedNotes.includes("Left One")}/>
            <Note onChange={(value) => updateNote("Left Two", value)} posX={357} posY={180} isSelected={selectedNotes.includes("Left Two")}/>
            <Note onChange={(value) => updateNote("Left Three", value)} posX={357} posY={88} isSelected={selectedNotes.includes("Left Three")}/>
            <Note onChange={(value) => updateNote("Right One", value)} posX={700} posY={487} isSelected={selectedNotes.includes("Right One")}/>
            <Note onChange={(value) => updateNote("Right Two", value)} posX={700} posY={380} isSelected={selectedNotes.includes("Right Two")}/>
            <Note onChange={(value) => updateNote("Right Three", value)} posX={700} posY={273} isSelected={selectedNotes.includes("Right Three")}/>
            <Note onChange={(value) => updateNote("Right Four", value)} posX={700} posY={166} isSelected={selectedNotes.includes("Right Four")}/>
            <Note onChange={(value) => updateNote("Right Five", value)} posX={700} posY={59} isSelected={selectedNotes.includes("Right Five")}/>
        </div>
    )
}

export default NoteSelector
