"use strict";

import {initUnits} from "./units.js";
import {start, update, handleInputs} from "./engine.js";

const canvas = document.getElementById("myCanvas");
const c = canvas.getContext("2d");

initUnits(canvas);

start(window, canvas);

handleInputs(window, canvas);

update(canvas, c);