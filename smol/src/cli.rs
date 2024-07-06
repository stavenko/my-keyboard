use std::path::PathBuf;

use clap::Parser;

#[derive(Parser)]
pub struct Command {
    #[arg(long)]
    pub output_path: PathBuf,
}
