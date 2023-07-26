# Usage
## Histogram mode
A histogram mode is provided for the meter. This affects the EBU_I and EBU_LRA specific methods, which are all loudnessGlobal* and loudnessRange* methods.

The histogram mode stores the blocks used for the gating in a histogram instead of a list. This sacrifices a bit of accuracy for constant time evalutation.

### Accuracy
The bin size of the histogram is 0.1 LU, which would entail an accuracy of 0.05 LU For the integrated and loudness range calculations you will for most normal audio experience less accuracy loss
than this, since the averaging will nicely smooth out the binning. Very narrowband signal, or a mix of such, will be the most negativly affected.

Median loudness measurments will naturally be more suseptible to error, as it selects a single bin (or in rare cases average of two adjecent).
For most use cases of median loudness this doesn't matter.

### Value range
As with the list based approach, anything below the absolute threshold of -70 LUFS is discarded.
The upper limit is (+)30 LUFS (with anything above being clamped to this), and should thus not cause any issues.

### Speed compared to block list
For very short audio, measured in seconds, histogram is no faster, or even slower.

Speed of adding blocks is the same (within measureability) as the block list.

The non-histogram calculation-time is still measured in microseconds for audio under an hour on modern systems.
The calculation cost is dwarfed by the calculations of the blocks themselves, so when measuring the values
for the whole audio there is no point in using the histogram mode.
If using it as a live-meter, where you query the global value frequently, the cost quickly starts
to become significant, and using the histogram mode would be recommended.
