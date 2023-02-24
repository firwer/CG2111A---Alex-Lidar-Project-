/**
 * @file server.js
 * 
 * The Express server used to handle and plot data sent from RPLidar.
 * To use, run `npm i` to install dependencies, then run `node server.js`.
 * The server will then listen on port 4000, and you can open the frontend at
 * http://localhost:4000
 */

import express from "express";
import formidable from "formidable";
import { fileURLToPath } from "url";
import { dirname } from "path";
import { randomBytes } from "crypto";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const app = express();
const port = process.argv[2] || 4000;

let value = [];
let clients = [];

const parseForm = (req) =>
    new Promise((resolve, reject) => {
        const form = formidable({ multiples: true });
        form.parse(req, (err, fields, files) => {
            if (err) {
                reject(err);
                return;
            }
            resolve({ fields, files });
        });
    });

app.get("/data", (req, res) => {
    const id = randomBytes(4).toString("hex");
    console.log(`${id} connected`);
    const headers = {
        "Content-Type": "text/event-stream",
        Connection: "keep-alive",
        "Cache-Control": "no-cache",
    };
    res.writeHead(200, headers);
    res.write(`data: ${JSON.stringify(value)}\n\n`);
    clients.push({ id, res });
    req.on("close", () => {
        console.log(`${id} disconnected`);
        clients = clients.filter((client) => client.id !== id);
    });
});

app.post("/data", async (req, res) => {
    console.log("POST /data");
    const { data } = (await parseForm(req)).fields;
    clients.forEach((client) => client.res.write(`data: ${data}\n\n`));
    res.status(200).end();
});

app.get("/", (_, res) => res.status(200).sendFile(__dirname + "/index.html"));

app.listen(port, () => console.log(`Server is running on port ${port}`));
