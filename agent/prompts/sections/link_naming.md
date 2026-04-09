<link_naming>
- Link names are part of the deliverable quality bar: keep them concise, semantic, and grounded in the object's intrinsic frame rather than arbitrary or state-based labels.
- Give each link an extremely concise semantic name: ideally just the part name, plus a short intrinsic location or shape cue only when needed to distinguish similar parts.
- Keep every link name to a single underscore-joined string with at most 5 words.
- Do not encode articulation state in link names. Avoid state words such as `open`, `closed`, `extended`, `pulled_out`, `ajar`, `tilted`, or `rotated`.
- Prefer names that say what the part is and, when helpful, what shape it has.
- Use location words only when the object has a meaningful canonical orientation or another clear object-intrinsic reference frame.
- When similar parts are reliably distinguishable, prefer object-intrinsic spatial cues such as `front_handle` or `side_support`.
- Do not invent `left`, `right`, `front`, or `back` distinctions for symmetric or orientation-ambiguous objects. Some objects have only a partial intrinsic frame: a humanoid part can be `left_arm`, but the two doors of a symmetric cabinet usually should not be `left_door` and `right_door`.
- If only part of the intrinsic frame is meaningful, use only that part. For example, if `front` and `back` are meaningful but `left` and `right` are ambiguous, use `front_*` or `rear_*` when needed and do not force side labels.
- If repeated parts are semantically identical and not intrinsically distinguishable, reuse the same base name and add numeric suffixes such as `door_0`, `door_1`. For 2D repeated layouts, names like `key_0_0`, `key_0_1` are acceptable.
</link_naming>
