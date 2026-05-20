from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.350
CASE_D = 0.162
CASE_BOTTOM_Z = 0.010
CASE_H = 0.019
CASE_TOP_Z = CASE_BOTTOM_Z + CASE_H
PITCH = 0.019
KEY_H = 0.007
SWITCH_H = 0.003


def _keycap_geometry(width: float, depth: float, height: float = KEY_H) -> MeshGeometry:
    """A simple low-profile keycap: wider bottom, smaller raised top."""
    top_w = max(width - 0.0040, width * 0.76)
    top_d = max(depth - 0.0040, depth * 0.76)
    geom = MeshGeometry()
    bottom = [
        (-width / 2, -depth / 2, 0.0),
        (width / 2, -depth / 2, 0.0),
        (width / 2, depth / 2, 0.0),
        (-width / 2, depth / 2, 0.0),
    ]
    top = [
        (-top_w / 2, -top_d / 2, height),
        (top_w / 2, -top_d / 2, height),
        (top_w / 2, top_d / 2, height),
        (-top_w / 2, top_d / 2, height),
    ]
    for vertex in bottom + top:
        geom.add_vertex(*vertex)

    # bottom, top, and four sloped sides
    for face in (
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ):
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_macro_keyboard")

    mat_case = model.material("matte_graphite", rgba=(0.020, 0.022, 0.026, 1.0))
    mat_deck = model.material("stepped_deck", rgba=(0.055, 0.058, 0.064, 1.0))
    mat_socket = model.material("switch_socket_black", rgba=(0.010, 0.011, 0.013, 1.0))
    mat_key = model.material("charcoal_keycaps", rgba=(0.12, 0.125, 0.135, 1.0))
    mat_macro = model.material("blue_macro_keycaps", rgba=(0.05, 0.18, 0.33, 1.0))
    mat_arrow = model.material("warm_gray_arrows", rgba=(0.20, 0.205, 0.215, 1.0))
    mat_legend = model.material("pale_legends", rgba=(0.86, 0.88, 0.84, 1.0))
    mat_rubber = model.material("dark_rubber", rgba=(0.018, 0.018, 0.018, 1.0))
    mat_metal = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))

    key_meshes: dict[tuple[float, float], object] = {}

    def key_mesh(width: float, depth: float):
        key = (round(width, 4), round(depth, 4))
        if key not in key_meshes:
            key_meshes[key] = mesh_from_geometry(
                _keycap_geometry(width, depth),
                f"keycap_{int(round(width * 1000))}x{int(round(depth * 1000))}",
            )
        return key_meshes[key]

    # One continuous, rounded shallow case body.
    case = model.part("case")
    case_shape = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, CASE_H)
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.002)
        .translate((0.0, 0.0, CASE_BOTTOM_Z + CASE_H / 2))
    )
    case.visual(
        mesh_from_cadquery(case_shape, "rounded_case"),
        material=mat_case,
        name="rounded_case",
    )

    # A right-rear corner pod carries the encoder shaft and oversized media knob.
    pod_x, pod_y = 0.132, 0.036
    pod_h = 0.006
    case.visual(
        Cylinder(radius=0.026, length=pod_h),
        origin=Origin(xyz=(pod_x, pod_y, CASE_TOP_Z + pod_h / 2)),
        material=mat_deck,
        name="corner_pod",
    )
    shaft_h = 0.020
    case.visual(
        Cylinder(radius=0.0040, length=shaft_h),
        origin=Origin(xyz=(pod_x, pod_y, CASE_TOP_Z + pod_h + shaft_h / 2)),
        material=mat_metal,
        name="encoder_shaft",
    )

    # Row decks are deliberately stepped rearward like a low-profile mechanical keyboard.
    row_ys = [0.054, 0.031, 0.008, -0.015, -0.038, -0.061]
    row_deck_zs = [CASE_TOP_Z + z for z in (0.0060, 0.0048, 0.0036, 0.0024, 0.0012, 0.0004)]
    for row_i, (row_y, deck_z) in enumerate(zip(row_ys, row_deck_zs)):
        strip_h = deck_z - CASE_TOP_Z + 0.0012
        case.visual(
            Box((0.315, 0.0205, strip_h)),
            origin=Origin(xyz=(-0.018, row_y, CASE_TOP_Z + strip_h / 2 - 0.0006)),
            material=mat_deck,
            name=f"row_step_{row_i}",
        )

    def add_key(
        name: str,
        x: float,
        y: float,
        *,
        row_index: int,
        width: float = 0.0162,
        depth: float = 0.0162,
        material: Material = mat_key,
        legend_width: float | None = None,
    ) -> None:
        deck_z = row_deck_zs[row_index]
        switch_w = max(width - 0.0040, 0.008)
        switch_d = max(depth - 0.0040, 0.008)
        case.visual(
            Box((switch_w, switch_d, SWITCH_H + 0.0008)),
            origin=Origin(xyz=(x, y, deck_z + SWITCH_H / 2 - 0.0004)),
            material=mat_socket,
            name=f"{name}_socket",
        )

        key = model.part(name)
        key.visual(
            key_mesh(width, depth),
            origin=Origin(),
            material=material,
            name="cap",
        )
        lw = legend_width if legend_width is not None else min(width * 0.36, 0.006)
        key.visual(
            Box((lw, 0.0012, 0.00045)),
            origin=Origin(xyz=(0.0, 0.0015, KEY_H + 0.00010)),
            material=mat_legend,
            name="legend_mark",
        )
        model.articulation(
            f"case_to_{name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, y, deck_z + SWITCH_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.08, lower=-0.0040, upper=0.0),
        )

    # Main alphanumeric block.
    main_x0 = -0.112
    for row in range(5):
        y = row_ys[row]
        offset = (row % 3) * 0.0020
        for col in range(12):
            # Leave a clean corner-pod pocket above the arrow cluster.
            if row == 0 and col >= 10:
                continue
            if row == 1 and col == 11:
                continue
            x = main_x0 + col * PITCH + offset
            add_key(f"key_{row}_{col}", x, y, row_index=row)

    # Larger bottom row keys: modifiers and a long space bar.
    bottom_specs = [
        ("key_5_0", -0.112, 0.021),
        ("key_5_1", -0.087, 0.021),
        ("key_5_2", -0.060, 0.024),
        ("key_5_3", -0.019, 0.058),
        ("key_5_4", 0.034, 0.030),
        ("key_5_5", 0.070, 0.021),
        ("key_5_6", 0.095, 0.021),
    ]
    for name, x, width in bottom_specs:
        add_key(
            name,
            x,
            row_ys[5],
            row_index=5,
            width=width,
            depth=0.0162,
            legend_width=min(width * 0.25, 0.018),
        )

    # Extra left-side macro column.
    macro_x = -0.153
    for i, y in enumerate([0.054, 0.031, 0.008, -0.015, -0.038, -0.061]):
        add_key(
            f"macro_key_{i}",
            macro_x,
            y,
            row_index=i,
            width=0.0170,
            depth=0.0162,
            material=mat_macro,
        )

    # Inverted-T arrow cluster below the pod/knob.
    arrow_specs = [
        ("arrow_key_0", pod_x, -0.030, 4),
        ("arrow_key_1", pod_x - PITCH, -0.053, 5),
        ("arrow_key_2", pod_x, -0.053, 5),
        ("arrow_key_3", pod_x + PITCH, -0.053, 5),
    ]
    for name, x, y, row_i in arrow_specs:
        add_key(name, x, y, row_index=row_i, material=mat_arrow)

    # Oversized clipped-on rotary media encoder knob.
    knob = model.part("media_knob")
    knob_geometry = KnobGeometry(
        0.034,
        0.018,
        body_style="faceted",
        base_diameter=0.037,
        top_diameter=0.028,
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=20, depth=0.0008, width=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=35.0),
        bore=KnobBore(style="round", diameter=0.010),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geometry, "media_knob"),
        origin=Origin(),
        material=mat_metal,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.0188, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=mat_rubber,
        name="retainer_skirt",
    )
    model.articulation(
        "case_to_media_knob",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=knob,
        origin=Origin(xyz=(pod_x, pod_y, CASE_TOP_Z + pod_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    # Two fold-out rear feet under the back edge.
    def add_rear_foot(index: int, x: float) -> None:
        y = 0.066
        z = CASE_BOTTOM_Z - 0.001
        # Parent-side hinge ears are part of the case and hang below the shell.
        for side in (-1, 1):
            case.visual(
                Box((0.006, 0.010, 0.008)),
                origin=Origin(xyz=(x + side * 0.030, y, CASE_BOTTOM_Z - 0.004)),
                material=mat_case,
                name=f"rear_foot_{index}_ear_{side}",
            )
        foot = model.part(f"rear_foot_{index}")
        foot.visual(
            Cylinder(radius=0.0040, length=0.055),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=mat_rubber,
            name="hinge_sleeve",
        )
        foot.visual(
            Box((0.052, 0.038, 0.0045)),
            origin=Origin(xyz=(0.0, -0.020, -0.0025)),
            material=mat_rubber,
            name="folding_pad",
        )
        foot.visual(
            Box((0.040, 0.003, 0.0012)),
            origin=Origin(xyz=(0.0, -0.037, -0.0050)),
            material=mat_deck,
            name="grip_ridge",
        )
        model.articulation(
            f"case_to_rear_foot_{index}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
        )

    add_rear_foot(0, -0.105)
    add_rear_foot(1, 0.105)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    knob = object_model.get_part("media_knob")
    knob_joint = object_model.get_articulation("case_to_media_knob")

    ctx.expect_within(
        knob,
        case,
        axes="xy",
        inner_elem="knob_cap",
        outer_elem="corner_pod",
        margin=0.009,
        name="media knob is centered on the corner pod",
    )
    ctx.expect_overlap(
        knob,
        case,
        axes="z",
        elem_a="knob_cap",
        elem_b="encoder_shaft",
        min_overlap=0.010,
        name="encoder shaft remains inserted through knob bore",
    )

    key_joint = object_model.get_articulation("case_to_key_2_4")
    sample_key = object_model.get_part("key_2_4")
    rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({key_joint: -0.004}):
        pressed_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "sample key plunges downward",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - 0.0035,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    rest_knob = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: math.pi / 2}):
        turned_knob = ctx.part_world_position(knob)
        ctx.expect_within(
            knob,
            case,
            axes="xy",
            inner_elem="knob_cap",
            outer_elem="corner_pod",
            margin=0.009,
            name="rotated media knob stays on pod",
        )
    ctx.check(
        "media knob rotates without lifting",
        rest_knob is not None
        and turned_knob is not None
        and abs(turned_knob[2] - rest_knob[2]) < 1e-6,
        details=f"rest={rest_knob}, turned={turned_knob}",
    )

    for index in (0, 1):
        foot = object_model.get_part(f"rear_foot_{index}")
        hinge = object_model.get_articulation(f"case_to_rear_foot_{index}")
        rest_aabb = ctx.part_world_aabb(foot)
        with ctx.pose({hinge: 1.0}):
            open_aabb = ctx.part_world_aabb(foot)
        ctx.check(
            f"rear foot {index} folds downward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] < rest_aabb[0][2] - 0.010,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
