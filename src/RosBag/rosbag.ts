import * as vscode from "vscode";
import { execSync, spawn } from "child_process";

const topics: string[] = [];

/**
 * 
 * @param inputBag 
 * @param output 
 * @param options 
 */
export async function filterBag(inputBag: vscode.Uri, output: vscode.Uri, options: string[]) {
    let filter = spawn('rosbag', ['filter', inputBag.fsPath, output.fsPath, ...options]);
}

// /**
//  * Plays a bag
//  * @param bags The chosen bag or bags 
//  * @param options The given bags
//  */
// export async function playBag(bag: vscode.Uri | Bag, options: string[]) {
//     if (bag instanceof vscode.Uri) {
//         bag = await open(bag.fsPath);
//     }

//     bag.readMessages({}, result => {
//         console.log(result);
//     });
// }

/**
 * Grabs the topics from a bag
 * @param bag The bag to grab topics from
 */
export function getTopics(bag: vscode.Uri) {
    // TODO: Do we want the rest of the metadata?
    let info = execSync(`rosbag info ${bag.fsPath}`).toString().split('\n');
    const regexp = /\/\S+\s+\d+ msgs?.*/g;
    console.log(info);
    const topics = [];
    for (let line of info) {
        const match = line.match(regexp);
        if (match) {
            topics.push(match[0].split(/\s/)[0]);
        }
    }

    console.log(topics);

    return topics;
}
