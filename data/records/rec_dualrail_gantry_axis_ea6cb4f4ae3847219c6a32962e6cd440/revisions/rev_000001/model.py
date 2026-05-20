from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


DECK_LENGTH = 1.08
DECK_WIDTH = 0.56
DECK_THICKNESS = 0.05
SIDE_FRAME_LENGTH = 0.98
SIDE_FRAME_WIDTH = 0.05
SIDE_FRAME_HEIGHT = 0.085
SIDE_FRAME_Y = 0.24

DECK_RAIL_Y = 0.165
RAIL_PAD_LENGTH = 0.90
RAIL_PAD_WIDTH = 0.066
RAIL_PAD_HEIGHT = 0.008
DECK_RAIL_LENGTH = 0.90
DECK_RAIL_WIDTH = 0.028
DECK_RAIL_HEIGHT = 0.014
RAIL_TOP_Z = RAIL_PAD_HEIGHT + DECK_RAIL_HEIGHT

BRIDGE_SHOE_LENGTH = 0.30
BRIDGE_SHOE_WIDTH = 0.062
BRIDGE_SHOE_HEIGHT = 0.036
BRIDGE_UPRIGHT_DEPTH = 0.14
BRIDGE_UPRIGHT_WIDTH = 0.046
BRIDGE_UPRIGHT_HEIGHT = 0.25
BRIDGE_UPRIGHT_Y = 0.188
BRIDGE_BEAM_LENGTH = 0.44
BRIDGE_BEAM_DEPTH = 0.106
BRIDGE_BEAM_HEIGHT = 0.142
BRIDGE_BEAM_CENTER_Z = 0.266
BRIDGE_TRAVEL = 0.23

BEAM_FRONT_FACE_X = BRIDGE_BEAM_DEPTH / 2.0
BEAM_RAIL_EXPOSED = 0.014
BEAM_RAIL_HEIGHT = 0.018
BEAM_RAIL_LENGTH = 0.38
LOWER_BEAM_RAIL_Z = 0.224
UPPER_BEAM_RAIL_Z = 0.288
TRUCK_JOINT_X = BEAM_FRONT_FACE_X + BEAM_RAIL_EXPOSED
TRUCK_JOINT_Z = (LOWER_BEAM_RAIL_Z + UPPER_BEAM_RAIL_Z) / 2.0

TRUCK_GUIDE_DEPTH = 0.032
TRUCK_GUIDE_LENGTH = 0.104
TRUCK_GUIDE_HEIGHT = 0.040
TRUCK_GUIDE_Z_OFFSET = (UPPER_BEAM_RAIL_Z - LOWER_BEAM_RAIL_Z) / 2.0
TRUCK_BODY_REAR_DEPTH = 0.026
TRUCK_BODY_REAR_WIDTH = 0.082
TRUCK_BODY_REAR_HEIGHT = 0.128
TRUCK_BODY_CORE_DEPTH = 0.056
TRUCK_BODY_CORE_WIDTH = 0.094
TRUCK_BODY_CORE_HEIGHT = 0.148
TRUCK_FRONT_PLATE_DEPTH = 0.018
TRUCK_FRONT_PLATE_WIDTH = 0.116
TRUCK_FRONT_PLATE_HEIGHT = 0.108
TRUCK_TRAVEL = 0.124

SAME_PART_EMBED = 0.001


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _xcyl(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def _zcyl(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _combine(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _build_base_body() -> cq.Workplane:
    deck = _box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS), (0.0, 0.0, -DECK_THICKNESS / 2.0))
    relief = _box((0.74, 0.21, 0.012), (0.0, 0.0, -0.006))
    deck = deck.cut(relief)

    side_frames: list[cq.Workplane] = []
    for y_pos in (-SIDE_FRAME_Y, SIDE_FRAME_Y):
        frame = _box(
            (SIDE_FRAME_LENGTH, SIDE_FRAME_WIDTH, SIDE_FRAME_HEIGHT),
            (0.0, y_pos, SIDE_FRAME_HEIGHT / 2.0),
        )
        window = _box((0.70, SIDE_FRAME_WIDTH * 0.72, 0.040), (0.0, y_pos, 0.048))
        frame = frame.cut(window)
        side_frames.append(frame)

    end_plinths = [
        _box((0.060, 0.10, 0.020), (-0.49, 0.0, 0.010)),
        _box((0.060, 0.10, 0.020), (0.49, 0.0, 0.010)),
    ]
    return _combine([deck, *side_frames, *end_plinths])


def _build_base_guides() -> cq.Workplane:
    guide_shapes: list[cq.Workplane] = []
    for y_pos in (-DECK_RAIL_Y, DECK_RAIL_Y):
        pad = _box(
            (RAIL_PAD_LENGTH, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT + SAME_PART_EMBED),
            (0.0, y_pos, (RAIL_PAD_HEIGHT - SAME_PART_EMBED) / 2.0),
        )
        rail = _box(
            (DECK_RAIL_LENGTH, DECK_RAIL_WIDTH, DECK_RAIL_HEIGHT + SAME_PART_EMBED),
            (0.0, y_pos, RAIL_PAD_HEIGHT + (DECK_RAIL_HEIGHT - SAME_PART_EMBED) / 2.0),
        )
        guide_shapes.extend([pad, rail])
    return _combine(guide_shapes)


def _build_base_trim() -> cq.Workplane:
    trim_shapes: list[cq.Workplane] = [
        _box((0.80, 0.018, 0.003), (0.0, 0.0, 0.0015)),
        _box((0.78, 0.016, 0.003), (0.0, -0.108, 0.0015)),
        _box((0.78, 0.016, 0.003), (0.0, 0.108, 0.0015)),
        _box((0.008, 0.020, 0.032), (-0.43, -0.090, 0.016)),
        _box((0.008, 0.020, 0.032), (-0.43, 0.090, 0.016)),
    ]

    for x_pos in (-0.47, 0.47):
        for y_pos in (-DECK_RAIL_Y, DECK_RAIL_Y):
            trim_shapes.append(_box((0.022, 0.050, 0.026), (x_pos, y_pos, 0.013)))

    return _combine(trim_shapes)


def _build_bridge_body() -> cq.Workplane:
    beam = _box(
        (BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_HEIGHT),
        (0.0, 0.0, BRIDGE_BEAM_CENTER_Z),
    )
    beam_pocket = _box((0.030, 0.27, 0.036), (0.018, 0.0, TRUCK_JOINT_Z))
    beam = beam.cut(beam_pocket)

    shapes: list[cq.Workplane] = [beam]
    for y_pos in (-BRIDGE_UPRIGHT_Y, BRIDGE_UPRIGHT_Y):
        upright = _box(
            (BRIDGE_UPRIGHT_DEPTH, BRIDGE_UPRIGHT_WIDTH, BRIDGE_UPRIGHT_HEIGHT + SAME_PART_EMBED),
            (
                0.0,
                y_pos,
                BRIDGE_SHOE_HEIGHT + (BRIDGE_UPRIGHT_HEIGHT - SAME_PART_EMBED) / 2.0,
            ),
        )
        upright_window = _box((0.070, BRIDGE_UPRIGHT_WIDTH * 1.2, 0.108), (0.0, y_pos, 0.160))
        upright = upright.cut(upright_window)
        gusset_front = _box((0.026, 0.038, 0.072), (0.046, y_pos, 0.070))
        gusset_rear = _box((0.026, 0.038, 0.072), (-0.046, y_pos, 0.070))
        shapes.extend([upright, gusset_front, gusset_rear])

    return _combine(shapes)


def _build_bridge_guides() -> cq.Workplane:
    shapes: list[cq.Workplane] = []
    for y_pos in (-DECK_RAIL_Y, DECK_RAIL_Y):
        shoe = _box(
            (BRIDGE_SHOE_LENGTH, BRIDGE_SHOE_WIDTH, BRIDGE_SHOE_HEIGHT),
            (0.0, y_pos, BRIDGE_SHOE_HEIGHT / 2.0),
        )
        front_cap = _box((0.072, BRIDGE_SHOE_WIDTH + 0.002, 0.006), (0.096, y_pos, BRIDGE_SHOE_HEIGHT + 0.003))
        rear_cap = _box((0.072, BRIDGE_SHOE_WIDTH + 0.002, 0.006), (-0.096, y_pos, BRIDGE_SHOE_HEIGHT + 0.003))
        shapes.extend([shoe, front_cap, rear_cap])
    return _combine(shapes)


def _build_bridge_fasteners() -> cq.Workplane:
    bolts: list[cq.Workplane] = []
    for y_pos in (-DECK_RAIL_Y, DECK_RAIL_Y):
        for x_pos in (-0.108, -0.084, 0.084, 0.108):
            for y_off in (-0.016, 0.016):
                bolts.append(_zcyl(0.005, 0.006, (x_pos, y_pos + y_off, BRIDGE_SHOE_HEIGHT + 0.006)))
    return _combine(bolts)


def _build_beam_guides() -> cq.Workplane:
    rail_center_x = BEAM_FRONT_FACE_X + (BEAM_RAIL_EXPOSED - SAME_PART_EMBED) / 2.0
    return _combine(
        [
            _box(
                (BEAM_RAIL_EXPOSED + SAME_PART_EMBED, BEAM_RAIL_LENGTH, BEAM_RAIL_HEIGHT),
                (rail_center_x, 0.0, LOWER_BEAM_RAIL_Z),
            ),
            _box(
                (BEAM_RAIL_EXPOSED + SAME_PART_EMBED, BEAM_RAIL_LENGTH, BEAM_RAIL_HEIGHT),
                (rail_center_x, 0.0, UPPER_BEAM_RAIL_Z),
            ),
        ]
    )


def _build_bridge_trim() -> cq.Workplane:
    return _combine(
        [
            _box((0.008, 0.34, 0.042), (BEAM_FRONT_FACE_X + 0.004, 0.0, TRUCK_JOINT_Z)),
            _box((0.006, 0.022, 0.032), (BEAM_FRONT_FACE_X + 0.003, -0.176, TRUCK_JOINT_Z + 0.050)),
        ]
    )


def _build_truck_body() -> cq.Workplane:
    rear_bridge = _box(
        (TRUCK_BODY_REAR_DEPTH, TRUCK_BODY_REAR_WIDTH, TRUCK_BODY_REAR_HEIGHT),
        (TRUCK_BODY_REAR_DEPTH / 2.0, 0.0, 0.0),
    )
    core = _box((TRUCK_BODY_CORE_DEPTH, TRUCK_BODY_CORE_WIDTH, TRUCK_BODY_CORE_HEIGHT), (0.058, 0.0, 0.0))
    front_plate = _box((TRUCK_FRONT_PLATE_DEPTH, TRUCK_FRONT_PLATE_WIDTH, TRUCK_FRONT_PLATE_HEIGHT), (0.106, 0.0, 0.0))
    body = _combine([rear_bridge, core, front_plate])
    side_relief = _box((0.022, 0.030, 0.070), (0.078, -0.032, 0.0))
    body = body.cut(side_relief).cut(side_relief.translate((0.0, 0.064, 0.0)))
    return body


def _build_truck_guides() -> cq.Workplane:
    return _combine(
        [
            _box(
                (TRUCK_GUIDE_DEPTH, TRUCK_GUIDE_LENGTH, TRUCK_GUIDE_HEIGHT),
                (TRUCK_GUIDE_DEPTH / 2.0, 0.0, -TRUCK_GUIDE_Z_OFFSET),
            ),
            _box(
                (TRUCK_GUIDE_DEPTH, TRUCK_GUIDE_LENGTH, TRUCK_GUIDE_HEIGHT),
                (TRUCK_GUIDE_DEPTH / 2.0, 0.0, TRUCK_GUIDE_Z_OFFSET),
            ),
        ]
    )


def _build_truck_trim() -> cq.Workplane:
    return _combine(
        [
            _box((0.010, 0.066, 0.118), (0.110, 0.0, 0.0)),
            _box((0.006, 0.020, 0.030), (0.114, -0.055, -0.030)),
        ]
    )


def _build_truck_fasteners() -> cq.Workplane:
    bolts: list[cq.Workplane] = []
    for z_pos in (-TRUCK_GUIDE_Z_OFFSET, TRUCK_GUIDE_Z_OFFSET):
        for y_pos in (-0.026, 0.026):
            bolts.append(_xcyl(0.004, 0.006, (TRUCK_GUIDE_DEPTH + 0.003, y_pos, z_pos - 0.010)))
            bolts.append(_xcyl(0.004, 0.006, (TRUCK_GUIDE_DEPTH + 0.003, y_pos, z_pos + 0.010)))
    return _combine(bolts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metrology_bridge_axis")

    model.material("painted_base", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("bridge_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("guide_caps", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("trim_blue", rgba=(0.20, 0.34, 0.64, 1.0))
    model.material("truck_silver", rgba=(0.84, 0.85, 0.87, 1.0))
    model.material("zinc", rgba=(0.82, 0.84, 0.88, 1.0))

    base = model.part("base_deck")
    _add_mesh_visual(
        base,
        _build_base_body(),
        mesh_name="base_deck_body",
        material="painted_base",
        visual_name="base_body",
    )
    _add_mesh_visual(
        base,
        _build_base_guides(),
        mesh_name="base_deck_guides",
        material="rail_steel",
        visual_name="base_guides",
    )
    _add_mesh_visual(
        base,
        _build_base_trim(),
        mesh_name="base_deck_trim",
        material="guide_caps",
        visual_name="base_trim",
    )
    base.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, SIDE_FRAME_HEIGHT + DECK_THICKNESS)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, (SIDE_FRAME_HEIGHT - DECK_THICKNESS) / 2.0)),
    )

    bridge = model.part("bridge_carriage")
    _add_mesh_visual(
        bridge,
        _build_bridge_body(),
        mesh_name="bridge_body_mesh",
        material="bridge_aluminum",
        visual_name="bridge_body",
    )
    _add_mesh_visual(
        bridge,
        _build_bridge_guides(),
        mesh_name="bridge_guides_mesh",
        material="guide_caps",
        visual_name="bridge_guides",
    )
    _add_mesh_visual(
        bridge,
        _build_beam_guides(),
        mesh_name="beam_guides_mesh",
        material="rail_steel",
        visual_name="beam_guides",
    )
    _add_mesh_visual(
        bridge,
        _build_bridge_trim(),
        mesh_name="bridge_trim_mesh",
        material="trim_blue",
        visual_name="bridge_trim",
    )
    _add_mesh_visual(
        bridge,
        _build_bridge_fasteners(),
        mesh_name="bridge_fasteners_mesh",
        material="zinc",
        visual_name="bridge_fasteners",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.34, 0.48, 0.34)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    truck = model.part("center_truck")
    _add_mesh_visual(
        truck,
        _build_truck_body(),
        mesh_name="truck_body_mesh",
        material="truck_silver",
        visual_name="truck_body",
    )
    _add_mesh_visual(
        truck,
        _build_truck_guides(),
        mesh_name="truck_guides_mesh",
        material="guide_caps",
        visual_name="truck_guides",
    )
    _add_mesh_visual(
        truck,
        _build_truck_trim(),
        mesh_name="truck_trim_mesh",
        material="trim_blue",
        visual_name="truck_trim",
    )
    _add_mesh_visual(
        truck,
        _build_truck_fasteners(),
        mesh_name="truck_fasteners_mesh",
        material="zinc",
        visual_name="truck_fasteners",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.13, 0.12, 0.16)),
        mass=8.5,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
    )

    bridge_slide = model.articulation(
        "deck_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=2200.0,
            velocity=0.45,
        ),
    )
    truck_slide = model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(TRUCK_JOINT_X, 0.0, TRUCK_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRUCK_TRAVEL,
            upper=TRUCK_TRAVEL,
            effort=600.0,
            velocity=0.30,
        ),
    )

    model.meta["primary_articulations"] = [bridge_slide.name, truck_slide.name]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_deck")
    bridge = object_model.get_part("bridge_carriage")
    truck = object_model.get_part("center_truck")
    bridge_slide = object_model.get_articulation("deck_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        bridge,
        base,
        elem_a="bridge_guides",
        elem_b="base_guides",
        contact_tol=0.001,
        name="bridge shoes contact deck rails",
    )
    ctx.expect_contact(
        truck,
        bridge,
        elem_a="truck_guides",
        elem_b="beam_guides",
        contact_tol=0.001,
        name="truck guide blocks contact beam rails",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        min_gap=0.010,
        positive_elem="bridge_body",
        negative_elem="base_guides",
        name="bridge frame clears fixed deck guide stack",
    )
    ctx.expect_gap(
        truck,
        bridge,
        axis="x",
        max_penetration=0.0,
        positive_elem="truck_body",
        negative_elem="beam_guides",
        name="truck body does not penetrate beam guides",
    )

    with ctx.pose({bridge_slide: bridge_slide.motion_limits.lower}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            margin=0.002,
            inner_elem="bridge_guides",
            outer_elem="base_guides",
            name="bridge guides stay on rails at negative travel",
        )
    with ctx.pose({bridge_slide: bridge_slide.motion_limits.upper}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            margin=0.002,
            inner_elem="bridge_guides",
            outer_elem="base_guides",
            name="bridge guides stay on rails at positive travel",
        )
    with ctx.pose({truck_slide: truck_slide.motion_limits.lower}):
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            margin=0.002,
            inner_elem="truck_guides",
            outer_elem="beam_guides",
            name="truck guides stay on beam rails at negative travel",
        )
    with ctx.pose({truck_slide: truck_slide.motion_limits.upper}):
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            margin=0.002,
            inner_elem="truck_guides",
            outer_elem="beam_guides",
            name="truck guides stay on beam rails at positive travel",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=40)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
