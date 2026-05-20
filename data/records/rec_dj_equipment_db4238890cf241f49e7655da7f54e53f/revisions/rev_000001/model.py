from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_dj_mixer")

    deck_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    panel_black = model.material("brushed_black_panel", rgba=(0.03, 0.032, 0.035, 1.0))
    side_gray = model.material("dark_side_anodized", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("soft_rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))
    white = model.material("silk_screen_white", rgba=(0.92, 0.92, 0.86, 1.0))
    warm_gray = model.material("engraved_warm_gray", rgba=(0.45, 0.45, 0.42, 1.0))
    metal = model.material("gunmetal_knurled", rgba=(0.18, 0.18, 0.17, 1.0))
    eq_blue = model.material("eq_blue_marks", rgba=(0.10, 0.33, 0.95, 1.0))
    eq_red = model.material("eq_red_marks", rgba=(0.85, 0.10, 0.08, 1.0))

    width = 0.58
    depth = 0.36
    base_h = 0.052
    panel_h = 0.006
    panel_z = base_h - 0.0005
    top_z = panel_z + panel_h

    base_profile = rounded_rect_profile(width, depth, 0.026, corner_segments=10)
    panel_profile = rounded_rect_profile(width - 0.030, depth - 0.030, 0.018, corner_segments=10)
    base_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(base_profile, base_h),
        "rounded_mixer_body",
    )
    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(panel_profile, panel_h),
        "raised_control_panel",
    )

    deck = model.part("deck")
    deck.visual(base_mesh, material=side_gray, name="rounded_body")
    deck.visual(panel_mesh, origin=Origin(xyz=(0.0, 0.0, panel_z)), material=panel_black, name="top_panel")

    # Low rubber feet are slightly embedded into the enclosure bottom so they read
    # as mounted rather than floating.
    for i, (x, y) in enumerate(
        (
            (-0.235, -0.140),
            (0.235, -0.140),
            (-0.235, 0.140),
            (0.235, 0.140),
        )
    ):
        deck.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    # Reusable meshes for the control caps and panel dress rings.
    eq_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.027,
            0.018,
            body_style="faceted",
            base_diameter=0.029,
            top_diameter=0.021,
            edge_radius=0.0006,
            grip=KnobGrip(style="ribbed", count=14, depth=0.00055, width=0.0012),
            center=False,
        ),
        "small_eq_knob_cap",
    )
    volume_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.041,
            0.024,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.046, 0.0045, flare=0.05, chamfer=0.0008),
            grip=KnobGrip(style="fluted", count=20, depth=0.00085),
            center=False,
        ),
        "channel_volume_knob_cap",
    )
    crossfader_mesh = mesh_from_geometry(
        KnobGeometry(
            0.062,
            0.030,
            body_style="skirted",
            top_diameter=0.050,
            skirt=KnobSkirt(0.078, 0.006, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="knurled", count=36, depth=0.0007, helix_angle_deg=16.0),
            center=False,
        ),
        "central_crossfader_knob_cap",
    )
    eq_ring_mesh = mesh_from_geometry(TorusGeometry(radius=0.0180, tube=0.0008), "eq_knob_collar")
    volume_ring_mesh = mesh_from_geometry(TorusGeometry(radius=0.0290, tube=0.0011), "volume_knob_collar")
    cross_ring_mesh = mesh_from_geometry(TorusGeometry(radius=0.0470, tube=0.0014), "crossfader_scale_ring")

    def add_top_box(
        name: str,
        x: float,
        y: float,
        sx: float,
        sy: float,
        material,
        *,
        yaw: float = 0.0,
        thickness: float = 0.0008,
    ) -> None:
        deck.visual(
            Box((sx, sy, thickness)),
            origin=Origin(xyz=(x, y, top_z - 0.0002 + thickness / 2.0), rpy=(0.0, 0.0, yaw)),
            material=material,
            name=name,
        )

    def add_tick_group(
        prefix: str,
        cx: float,
        cy: float,
        radius: float,
        angles_deg: tuple[float, ...],
        material,
        *,
        length: float = 0.006,
        width_tick: float = 0.0018,
    ) -> None:
        for idx, deg in enumerate(angles_deg):
            angle = math.radians(deg)
            x = cx + math.sin(angle) * radius
            y = cy + math.cos(angle) * radius
            add_top_box(
                f"{prefix}_tick_{idx}",
                x,
                y,
                width_tick,
                length,
                material,
                yaw=-angle,
                thickness=0.0007,
            )

    def add_ring(name: str, mesh, x: float, y: float, material, tube: float) -> None:
        deck.visual(
            mesh,
            origin=Origin(xyz=(x, y, top_z - 0.0002 + tube)),
            material=material,
            name=name,
        )

    # Channel separators and printed legends. These are thin inlaid graphics,
    # not raised controls, and are slightly embedded into the top panel.
    for idx, x in enumerate((-0.1825, -0.095, 0.095, 0.1825)):
        add_top_box(f"channel_separator_{idx}", x, 0.028, 0.0013, 0.245, warm_gray)
    for idx, x in enumerate((-0.225, -0.140, 0.140, 0.225)):
        add_top_box(f"channel_label_bar_{idx}", x, 0.159, 0.050, 0.006, white)
        add_top_box(f"channel_color_bar_{idx}", x, 0.148, 0.036, 0.004, eq_blue if idx < 2 else eq_red)
    add_top_box("crossfader_label_bar", 0.0, -0.166, 0.070, 0.006, white)
    add_top_box("crossfader_center_mark", 0.0, -0.090, 0.003, 0.018, white)

    channel_xs = (-0.225, -0.140, 0.140, 0.225)
    eq_rows = (
        ("hi", 0.118, eq_red),
        ("mid", 0.060, white),
        ("low", 0.002, eq_blue),
    )

    continuous_limits = MotionLimits(effort=0.45, velocity=8.0)

    def make_rotary_knob(
        part_name: str,
        joint_name: str,
        x: float,
        y: float,
        cap_mesh,
        cap_height: float,
        indicator_len: float,
        indicator_width: float,
        indicator_offset: float,
        ring_mesh,
        ring_tube: float,
        ring_material,
    ) -> None:
        add_ring(f"{part_name}_collar", ring_mesh, x, y, ring_material, ring_tube)
        knob = model.part(part_name)
        knob.visual(cap_mesh, material=deck_black, name="cap")
        knob.visual(
            Box((indicator_width, indicator_len, cap_height + 0.0040)),
            origin=Origin(
                xyz=(0.0, indicator_offset, (cap_height + 0.0040) / 2.0),
            ),
            material=white,
            name="pointer_line",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=knob,
            origin=Origin(xyz=(x, y, top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=continuous_limits,
        )

    for col, x in enumerate(channel_xs):
        for row, (eq_name, y, _mat) in enumerate(eq_rows):
            part_name = f"eq_{col}_{row}"
            make_rotary_knob(
                part_name,
                f"deck_to_{part_name}",
                x,
                y,
                eq_knob_mesh,
                0.018,
                0.011,
                0.0018,
                0.0042,
                eq_ring_mesh,
                0.0008,
                metal,
            )
            add_tick_group(
                f"{part_name}_scale",
                x,
                y,
                0.026,
                (-105, -52, 0, 52, 105),
                _mat,
                length=0.0045,
                width_tick=0.0012,
            )

        part_name = f"volume_{col}"
        y = -0.105
        make_rotary_knob(
            part_name,
            f"deck_to_{part_name}",
            x,
            y,
            volume_knob_mesh,
            0.024,
            0.017,
            0.0023,
            0.0068,
            volume_ring_mesh,
            0.0011,
            metal,
        )
        add_tick_group(
            f"{part_name}_scale",
            x,
            y,
            0.039,
            (-130, -85, -40, 0, 40, 85, 130),
            white,
            length=0.0065,
            width_tick=0.0015,
        )

    make_rotary_knob(
        "crossfader_knob",
        "deck_to_crossfader_knob",
        0.0,
        -0.105,
        crossfader_mesh,
        0.030,
        0.025,
        0.0032,
        0.0095,
        cross_ring_mesh,
        0.0014,
        metal,
    )
    add_tick_group(
        "crossfader_scale",
        0.0,
        -0.105,
        0.060,
        (-150, -112, -75, -38, 0, 38, 75, 112, 150),
        white,
        length=0.009,
        width_tick=0.002,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    top_panel_elem = "top_panel"

    volume_names = [f"volume_{idx}" for idx in range(4)]
    eq_names = [f"eq_{col}_{row}" for col in range(4) for row in range(3)]
    rotary_names = volume_names + eq_names + ["crossfader_knob"]
    rotary_joints = [object_model.get_articulation(f"deck_to_{name}") for name in rotary_names]

    ctx.check("four channel volume knobs", len(volume_names) == 4)
    ctx.check("three eq knobs per channel", len(eq_names) == 12)
    ctx.check("all controls are continuous", all(j.articulation_type == ArticulationType.CONTINUOUS for j in rotary_joints))
    ctx.check("all rotary axes are vertical", all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in rotary_joints))

    for name in rotary_names:
        knob = object_model.get_part(name)
        ctx.expect_gap(
            knob,
            deck,
            axis="z",
            positive_elem="cap",
            negative_elem=top_panel_elem,
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{name} cap seated on faceplate",
        )

    with ctx.pose({"deck_to_crossfader_knob": math.tau * 1.25}):
        crossfader = object_model.get_part("crossfader_knob")
        ctx.expect_gap(
            crossfader,
            deck,
            axis="z",
            positive_elem="cap",
            negative_elem=top_panel_elem,
            max_gap=0.001,
            max_penetration=0.0,
            name="crossfader remains seated during continuous rotation",
        )

    return ctx.report()


object_model = build_object_model()
