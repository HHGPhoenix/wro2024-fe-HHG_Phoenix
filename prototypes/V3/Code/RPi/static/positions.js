function drawBlockPositions() {
    let container = document.getElementsByClassName("container")[0];

    // Create a 3x3 grid of divs
    for (let i = 1; i < 4; i++) {
        for (let j = 1; j < 4; j++) {
            let div = document.createElement("div");
            div.classList.add("block");
            div.classList.add("block-" + i + "-" + j);
            container.appendChild(div);

            if ((i * j) % 2 == 0 && i * j != 4) {
                // Create a 4x3 grid of divs inside each block
                for (let k = 0; k < 2; k++) {
                    for (let l = 0; l < 3; l++) {
                        let innerDiv = document.createElement("div");
                        innerDiv.classList.add("inner-block");
                        innerDiv.classList.add("inner-block-" + k + "-" + l);
                        innerDiv.textContent = k + "-" + l;
                        div.appendChild(innerDiv);
                    }
                }
            }
        }
    }
}

async function updateData() {
    await fetch("/positions_feed")
        .then((response) => {
            return response.json();
        })
        .then((data) => {
            let positions = data;
            console.log(positions);
            updateBlockVisuals(positions);
        })
        .catch((error) => {
            console.error("Error:", error);
            //resetBlockVisuals();
        });
}

function resetBlockVisuals() {
    for (let i = 0; i < 4; i++) {
        let j;
        if (i == 3) {
            j = 2;
        } else if (i == 2) {
            j = 3;
        } else {
            j = i;
        }

        let innerblock1 = document.getElementsByClassName("inner-block-0-0")[j];
        let innerblock2 = document.getElementsByClassName("inner-block-0-1")[j];
        let innerblock3 = document.getElementsByClassName("inner-block-1-0")[j];
        let innerblock4 = document.getElementsByClassName("inner-block-1-1")[j];
        let innerblock5 = document.getElementsByClassName("inner-block-0-2")[j];
        let innerblock6 = document.getElementsByClassName("inner-block-1-2")[j];

        innerblock1.style.backgroundColor = "transparent";
        innerblock2.style.backgroundColor = "transparent";
        innerblock3.style.backgroundColor = "transparent";
        innerblock4.style.backgroundColor = "transparent";
        innerblock5.style.backgroundColor = "transparent";
        innerblock6.style.backgroundColor = "transparent";
    }
}

function updateBlockVisuals(positions) {
    for (let i = 0; i < 4; i++) {
        let j;
        if (i == 3) {
            j = 2;
        } else if (i == 2) {
            j = 3;
        } else {
            j = i;
        }

        if (positions[i]?.position == 1) {
            let innerblock1 =
                document.getElementsByClassName("inner-block-0-0")[j];
            let innerblock2 =
                document.getElementsByClassName("inner-block-1-0")[j];

            innerblock1.style.backgroundColor = positions[i].color;
            innerblock2.style.backgroundColor = positions[i].color;
        } else if (positions[i]?.position == 2) {
            let innerblock1 =
                document.getElementsByClassName("inner-block-0-1")[j];
            let innerblock2 =
                document.getElementsByClassName("inner-block-1-1")[j];

            innerblock1.style.backgroundColor = positions[i].color;
            innerblock2.style.backgroundColor = positions[i].color;
        } else if (positions[i]?.position == 3) {
            let innerblock1 =
                document.getElementsByClassName("inner-block-0-2")[j];
            let innerblock2 =
                document.getElementsByClassName("inner-block-1-2")[j];

            console.log(positions[i].color);
            innerblock1.style.backgroundColor = positions[i].color;
            innerblock2.style.backgroundColor = positions[i].color;
        }
    }
}

drawBlockPositions();
setInterval(updateData, 500);
