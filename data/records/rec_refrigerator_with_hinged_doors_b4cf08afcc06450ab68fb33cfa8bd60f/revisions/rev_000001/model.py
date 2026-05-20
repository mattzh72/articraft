from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HINGE_X = 0.395
HINGE_Y = -0.342
UPPER_DOOR_Z = 1.250
LOWER_DOOR_Z = 0.550
DIAL_X = 0.145
DIAL_Z = 1.585
PANEL_FRONT_Y = -0.356


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """A softly rounded rectangular solid in meters."""
    safe_radius = min(radius, width * 0.48, depth * 0.48)
    return cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(safe_radius)


def _door_panel_mesh(width: float, depth: float, height: float, radius: float, name: str):
    # Door panels are rounded in their front silhouette, so fillet the four
    # short edges running through the door thickness rather than the vertical
    # side edges that are limited by the shallow panel depth.
    safe_radius = min(radius, width * 0.45, height * 0.45)
    return mesh_from_cadquery(
        cq.Workplane("XY").box(width, depth, height).edges("|Y").fillet(safe_radius),
        name,
        tolerance=0.001,
        angular_tolerance=0.08,
    )


def _add_hinge_group(
    cabinet,
    *,
    z: float,
    clip_len: float,
    barrel_len: float,
    chrome: Material,
    stem: str,
) -> None:
    """Add the two fixed knuckles that capture one moving door clip."""
    barrel_radius = 0.027
    dz = 0.5 * (clip_len + barrel_len)
    for suffix, offset in (("lower", -dz), ("upper", dz)):
        center_z = z + offset
        cabinet.visual(
            Cylinder(radius=barrel_radius, length=barrel_len),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, center_z)),
            material=chrome,
            name=f"{stem}_{suffix}_barrel",
        )
        cabinet.visual(
            Box((0.064, 0.038, barrel_len)),
            origin=Origin(xyz=(HINGE_X - 0.030, HINGE_Y + 0.032, center_z)),
            material=chrome,
            name=f"{stem}_{suffix}_barrel_leaf",
        )


def _add_door_hinge_clip(
    door,
    *,
    z: float,
    length: float,
    chrome: Material,
    stem: str,
) -> None:
    door.visual(
        Cylinder(radius=0.023, length=length),
        origin=Origin(xyz=(0.0, 0.0, z)),
        material=chrome,
        name=f"{stem}_clip",
    )
    door.visual(
        Box((0.088, 0.022, length)),
        origin=Origin(xyz=(-0.044, -0.004, z)),
        material=chrome,
        name=f"{stem}_clip_leaf",
    )


def _add_retro_handle(
    door,
    *,
    x: float,
    z: float,
    length: float,
    chrome: Material,
    stem: str,
) -> None:
    door.visual(
        Cylinder(radius=0.018, length=length),
        origin=Origin(xyz=(x, -0.108, z)),
        material=chrome,
        name=f"{stem}_grip",
    )
    for suffix, dz in (("lower", -0.36 * length), ("upper", 0.36 * length)):
        door.visual(
            Box((0.052, 0.078, 0.035)),
            origin=Origin(xyz=(x, -0.070, z + dz)),
            material=chrome,
            name=f"{stem}_{suffix}_standoff",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_two_door_refrigerator")

    mint = model.material("mint_enamel", rgba=(0.52, 0.82, 0.78, 1.0))
    ivory = model.material("warm_ivory", rgba=(0.92, 0.86, 0.72, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.035, 0.038, 0.040, 1.0))
    black = model.material("black_print", rgba=(0.02, 0.02, 0.018, 1.0))
    red = model.material("red_pointer", rgba=(0.78, 0.12, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(
            _rounded_box(0.800, 0.600, 1.680, 0.060),
            "rounded_cabinet_body",
            tolerance=0.0015,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=mint,
        name="main_shell",
    )

    # Fixed upper control trim: the freezer door starts below it, so the
    # temperature dial is part of the stationary cabinet rather than the door.
    cabinet.visual(
        mesh_from_cadquery(
            _rounded_box(0.700, 0.065, 0.160, 0.025),
            "fixed_upper_trim_panel",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(-0.010, -0.3235, DIAL_Z)),
        material=ivory,
        name="upper_trim_panel",
    )
    cabinet.visual(
        Box((0.720, 0.070, 0.020)),
        origin=Origin(xyz=(-0.010, -0.330, 1.492)),
        material=chrome,
        name="freezer_trim_strip",
    )
    cabinet.visual(
        Box((0.720, 0.070, 0.018)),
        origin=Origin(xyz=(-0.010, -0.330, 1.018)),
        material=chrome,
        name="door_separator_strip",
    )
    cabinet.visual(
        Box((0.700, 0.070, 0.070)),
        origin=Origin(xyz=(-0.010, -0.330, 0.055)),
        material=chrome,
        name="toe_kick_plate",
    )
    for i, x in enumerate((-0.280, -0.160, -0.040, 0.080, 0.200)):
        cabinet.visual(
            Box((0.055, 0.010, 0.012)),
            origin=Origin(xyz=(x, -0.368, 0.060)),
            material=rubber,
            name=f"toe_vent_{i}",
        )

    # Four stubby chrome/rubber feet overlap the lower shell very slightly so
    # the cabinet reads as a supported appliance.
    for i, (x, y) in enumerate(((-0.300, -0.210), (0.300, -0.210), (-0.300, 0.210), (0.300, 0.210))):
        cabinet.visual(
            Cylinder(radius=0.040, length=0.045),
            origin=Origin(xyz=(x, y, -0.016)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Dial scale marks are printed on the fixed trim around the moving knob.
    for i, angle_deg in enumerate((-120, -75, -30, 30, 75, 120)):
        a = math.radians(angle_deg)
        tick_x = DIAL_X + 0.073 * math.sin(a)
        tick_z = DIAL_Z + 0.073 * math.cos(a)
        cabinet.visual(
            Box((0.006, 0.006, 0.025)),
            origin=Origin(
                xyz=(tick_x, PANEL_FRONT_Y - 0.002, tick_z),
                rpy=(0.0, a, 0.0),
            ),
            material=black,
            name=f"dial_tick_{i}",
        )
    cabinet.visual(
        Box((0.115, 0.006, 0.010)),
        origin=Origin(xyz=(DIAL_X, PANEL_FRONT_Y - 0.002, DIAL_Z - 0.105)),
        material=black,
        name="cold_label_bar",
    )

    # Fixed hinge knuckles on the cabinet side.  Each moving door clip is held
    # between an upper and lower barrel at the same hinge axis.
    for rel_z in (-0.130, 0.130):
        _add_hinge_group(
            cabinet,
            z=UPPER_DOOR_Z + rel_z,
            clip_len=0.055,
            barrel_len=0.040,
            chrome=chrome,
            stem=f"freezer_{'lower' if rel_z < 0 else 'upper'}",
        )
    for rel_z in (-0.320, 0.320):
        _add_hinge_group(
            cabinet,
            z=LOWER_DOOR_Z + rel_z,
            clip_len=0.090,
            barrel_len=0.060,
            chrome=chrome,
            stem=f"fresh_{'lower' if rel_z < 0 else 'upper'}",
        )

    freezer_door = model.part("freezer_door")
    freezer_door.visual(
        _door_panel_mesh(0.700, 0.065, 0.420, 0.038, "freezer_rounded_door"),
        origin=Origin(xyz=(-0.395, -0.016, 0.0)),
        material=ivory,
        name="freezer_panel",
    )
    freezer_door.visual(
        _door_panel_mesh(0.610, 0.010, 0.315, 0.026, "freezer_inset_panel"),
        origin=Origin(xyz=(-0.395, -0.053, 0.0)),
        material=mint,
        name="freezer_inset",
    )
    freezer_door.visual(
        Box((0.580, 0.006, 0.014)),
        origin=Origin(xyz=(-0.395, -0.061, -0.135)),
        material=chrome,
        name="freezer_chrome_band",
    )
    _add_retro_handle(
        freezer_door,
        x=-0.675,
        z=0.010,
        length=0.250,
        chrome=chrome,
        stem="freezer_handle",
    )
    for rel_z in (-0.130, 0.130):
        _add_door_hinge_clip(
            freezer_door,
            z=rel_z,
            length=0.055,
            chrome=chrome,
            stem=f"freezer_{'lower' if rel_z < 0 else 'upper'}",
        )

    fresh_door = model.part("fresh_door")
    fresh_door.visual(
        _door_panel_mesh(0.700, 0.065, 0.860, 0.040, "fresh_rounded_door"),
        origin=Origin(xyz=(-0.395, -0.016, 0.0)),
        material=ivory,
        name="fresh_panel",
    )
    fresh_door.visual(
        _door_panel_mesh(0.610, 0.010, 0.730, 0.030, "fresh_inset_panel"),
        origin=Origin(xyz=(-0.395, -0.053, 0.0)),
        material=mint,
        name="fresh_inset",
    )
    fresh_door.visual(
        Box((0.590, 0.006, 0.016)),
        origin=Origin(xyz=(-0.395, -0.061, 0.270)),
        material=chrome,
        name="fresh_chrome_band",
    )
    fresh_door.visual(
        Box((0.590, 0.006, 0.016)),
        origin=Origin(xyz=(-0.395, -0.061, -0.270)),
        material=chrome,
        name="fresh_lower_band",
    )
    _add_retro_handle(
        fresh_door,
        x=-0.675,
        z=0.040,
        length=0.520,
        chrome=chrome,
        stem="fresh_handle",
    )
    for rel_z in (-0.320, 0.320):
        _add_door_hinge_clip(
            fresh_door,
            z=rel_z,
            length=0.090,
            chrome=chrome,
            stem=f"fresh_{'lower' if rel_z < 0 else 'upper'}",
        )

    dial = model.part("temp_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.086,
                0.036,
                body_style="skirted",
                top_diameter=0.064,
                edge_radius=0.0012,
                skirt=KnobSkirt(0.104, 0.006, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "temperature_dial_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="dial_cap",
    )
    dial.visual(
        Box((0.008, 0.004, 0.050)),
        origin=Origin(xyz=(0.000, -0.039, 0.014)),
        material=red,
        name="dial_pointer",
    )

    model.articulation(
        "cabinet_to_freezer_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, UPPER_DOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_fresh_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=fresh_door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, LOWER_DOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_temp_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(DIAL_X, PANEL_FRONT_Y, DIAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    freezer = object_model.get_part("freezer_door")
    fresh = object_model.get_part("fresh_door")
    dial = object_model.get_part("temp_dial")
    freezer_hinge = object_model.get_articulation("cabinet_to_freezer_door")
    fresh_hinge = object_model.get_articulation("cabinet_to_fresh_door")
    dial_joint = object_model.get_articulation("cabinet_to_temp_dial")

    # The doors are overlaid in front of the rounded cabinet, not embedded in it.
    ctx.expect_gap(
        cabinet,
        freezer,
        axis="y",
        positive_elem="main_shell",
        negative_elem="freezer_panel",
        min_gap=0.025,
        name="freezer door clears cabinet front",
    )
    ctx.expect_gap(
        cabinet,
        fresh,
        axis="y",
        positive_elem="main_shell",
        negative_elem="fresh_panel",
        min_gap=0.025,
        name="fresh door clears cabinet front",
    )
    ctx.expect_overlap(
        freezer,
        cabinet,
        axes="xz",
        elem_a="freezer_panel",
        elem_b="main_shell",
        min_overlap=0.35,
        name="freezer door covers upper cabinet opening",
    )
    ctx.expect_overlap(
        fresh,
        cabinet,
        axes="xz",
        elem_a="fresh_panel",
        elem_b="main_shell",
        min_overlap=0.65,
        name="fresh door covers lower cabinet opening",
    )

    # Each moving clip is captured between fixed barrels at the cabinet side.
    for moving, lower_barrel, upper_barrel, door in (
        ("freezer_lower_clip", "freezer_lower_lower_barrel", "freezer_lower_upper_barrel", freezer),
        ("freezer_upper_clip", "freezer_upper_lower_barrel", "freezer_upper_upper_barrel", freezer),
        ("fresh_lower_clip", "fresh_lower_lower_barrel", "fresh_lower_upper_barrel", fresh),
        ("fresh_upper_clip", "fresh_upper_lower_barrel", "fresh_upper_upper_barrel", fresh),
    ):
        ctx.expect_contact(
            door,
            cabinet,
            elem_a=moving,
            elem_b=lower_barrel,
            contact_tol=0.001,
            name=f"{moving} sits on lower hinge barrel",
        )
        ctx.expect_contact(
            door,
            cabinet,
            elem_a=moving,
            elem_b=upper_barrel,
            contact_tol=0.001,
            name=f"{moving} is capped by upper hinge barrel",
        )

    ctx.expect_contact(
        dial,
        cabinet,
        elem_a="dial_cap",
        elem_b="upper_trim_panel",
        contact_tol=0.001,
        name="temperature dial mounts to fixed upper trim",
    )

    ctx.check(
        "freezer hinge is vertical",
        tuple(round(v, 3) for v in freezer_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={freezer_hinge.axis}",
    )
    ctx.check(
        "fresh hinge is vertical",
        tuple(round(v, 3) for v in fresh_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={fresh_hinge.axis}",
    )
    ctx.check(
        "dial rotates on front-back axis",
        abs(dial_joint.axis[1]) > 0.9 and abs(dial_joint.axis[0]) < 0.01 and abs(dial_joint.axis[2]) < 0.01,
        details=f"axis={dial_joint.axis}",
    )

    with ctx.pose({freezer_hinge: 1.2, fresh_hinge: 1.2, dial_joint: 1.0}):
        freezer_aabb = ctx.part_element_world_aabb(freezer, elem="freezer_panel")
        fresh_aabb = ctx.part_element_world_aabb(fresh, elem="fresh_panel")
        ctx.check(
            "freezer door swings outward",
            freezer_aabb is not None and freezer_aabb[0][1] < -0.74,
            details=f"aabb={freezer_aabb}",
        )
        ctx.check(
            "fresh door swings outward",
            fresh_aabb is not None and fresh_aabb[0][1] < -0.74,
            details=f"aabb={fresh_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
