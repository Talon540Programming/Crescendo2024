import * as React from "react";

interface NoteContainerProps {
    onChange: (selected: boolean) => void
    isSelected: boolean
    posX: number
    posY: number
}

const Note = ({onChange, isSelected, posX, posY}: NoteContainerProps) => {
    return (
        <div
            className={`note-container ${isSelected ? "selected" : ""}`}
            style={{position: "fixed", top: posY, left: posX}}
            onClick={() => onChange(!isSelected)}/>
    )
}

export default Note
