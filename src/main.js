"use strict";

import {initUnits} from "./units.js";
import {start, update, handleInputs} from "./engine.js";

const canvas = document.getElementById("engineCanvas");
const c = canvas.getContext("2d");

initUnits(canvas);

start(window, canvas, c);

handleInputs(window, canvas);

update(canvas, c);