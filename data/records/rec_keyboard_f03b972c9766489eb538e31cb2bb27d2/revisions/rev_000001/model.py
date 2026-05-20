from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


KEY_PITCH = 0.019
PLATE_TOP_Z = 0.022
KEY_TRAVEL = 0.004


def _rounded_box(width: float, depth: float, height: float, bevel: float) -> cq.Workplane:
    """A softened rectangular solid centered on the local origin."""
    body = cq.Workplane("XY").box(width, depth, height)
    if bevel > 0.0:
        body = body.edges("|Z").fillet(bevel)
        body = body.edges(">Z").fillet(min(bevel * 0.45, height * 0.18))
    return body


def _keycap_mesh(width: float, depth: float, height: float) -> cq.Workplane:
    """Slightly tapered keycap with a broad skirt and smaller top face."""
    top_width = max(width - 0.0026, width * 0.72)
    top_depth = max(depth - 0.0026, depth * 0.72)
    cap = (
        cq.Workplane("XY")
        .rect(width, depth)
        .workplane(offset=height)
        .rect(top_width, top_depth)
        .loft(combine=True)
    )
    return cap


def _plate_with_switch_openings(
    width: float,
    depth: float,
    thickness: float,
    openings: list[tuple[float, float, float, float]],
) -> cq.Workplane:
    """Inset top plate with rectangular clearance openings for moving stems."""
    plate = cq.Workplane("XY").box(width, depth, thickness)
    for x, y, hole_w, hole_d in openings:
        cutter = cq.Workplane("XY").box(hole_w, hole_d, thickness * 3.0).translate((x, y, 0.0))
        plate = plate.cut(cutter)
    return plate.edges("|Z").fillet(0.0012)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_mechanical_keyboard")

    case_mat = model.material("powder_coated_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    rim_mat = model.material("soft_black_case_rim", rgba=(0.018, 0.019, 0.021, 1.0))
    plate_mat = model.material("brushed_black_plate", rgba=(0.006, 0.007, 0.008, 1.0))
    key_mat = model.material("warm_gray_keycaps", rgba=(0.35, 0.35, 0.34, 1.0))
    mod_mat = model.material("charcoal_modifier_keys", rgba=(0.14, 0.145, 0.15, 1.0))
    accent_mat = model.material("orange_escape_key", rgba=(0.90, 0.32, 0.08, 1.0))
    stem_mat = model.material("black_switch_stems", rgba=(0.02, 0.02, 0.022, 1.0))

    case_w = 0.306
    case_d = 0.126
    base_h = 0.018
    rim_h = 0.009
    plate_w = 0.270
    plate_d = 0.097
    plate_t = 0.003

    cap_h = 0.009
    cap_bottom_z = 0.006
    key_w = 0.0165
    key_d = 0.0165
    stem_w = 0.0062
    stem_d = 0.0062
    stem_h = 0.008
    stem_center_z = stem_h * 0.5

    row_y = [0.038, 0.019, 0.0, -0.019, -0.038]
    col_x = [(i - 5.5) * KEY_PITCH for i in range(12)]

    normal_key_mesh = mesh_from_cadquery(_keycap_mesh(key_w, key_d, cap_h), "normal_tapered_keycap")
    space_key_w = 0.108
    space_key_mesh = mesh_from_cadquery(_keycap_mesh(space_key_w, key_d, cap_h), "spacebar_keycap")

    openings: list[tuple[float, float, float, float]] = []
    for y in row_y[:4]:
        for x in col_x:
            openings.append((x, y, 0.0105, 0.0105))
    for x in (col_x[0], col_x[1], col_x[2], col_x[9], col_x[10], col_x[11]):
        openings.append((x, row_y[4], 0.0105, 0.0105))
    openings.extend(
        [
            (0.0, row_y[4], 0.011, 0.0105),
            (-0.040, row_y[4], 0.0085, 0.0105),
            (0.040, row_y[4], 0.0085, 0.0105),
        ]
    )

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_rounded_box(case_w, case_d, base_h, 0.007), "rounded_case_shell"),
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
        material=case_mat,
        name="case_shell",
    )

    rim_y = (case_d + plate_d) * 0.25
    rim_strip_d = (case_d - plate_d) * 0.5 + 0.006
    rim_strip_w = case_w
    rim_z = base_h + rim_h * 0.5
    case.visual(
        Box((rim_strip_w, rim_strip_d, rim_h)),
        origin=Origin(xyz=(0.0, rim_y, rim_z)),
        material=rim_mat,
        name="rear_rim",
    )
    case.visual(
        Box((rim_strip_w, rim_strip_d, rim_h)),
        origin=Origin(xyz=(0.0, -rim_y, rim_z)),
        material=rim_mat,
        name="front_rim",
    )
    rim_x = (case_w + plate_w) * 0.25
    rim_strip_w_side = (case_w - plate_w) * 0.5 + 0.006
    case.visual(
        Box((rim_strip_w_side, plate_d + 0.006, rim_h)),
        origin=Origin(xyz=(rim_x, 0.0, rim_z)),
        material=rim_mat,
        name="side_rim_0",
    )
    case.visual(
        Box((rim_strip_w_side, plate_d + 0.006, rim_h)),
        origin=Origin(xyz=(-rim_x, 0.0, rim_z)),
        material=rim_mat,
        name="side_rim_1",
    )
    case.visual(
        mesh_from_cadquery(
            _plate_with_switch_openings(plate_w, plate_d, plate_t, openings),
            "recessed_switch_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z - plate_t * 0.5)),
        material=plate_mat,
        name="top_plate",
    )
    socket_h = PLATE_TOP_Z - base_h
    for idx, (x, y, hole_w, hole_d) in enumerate(openings):
        case.visual(
            Box((min(0.008, hole_w - 0.001), min(0.008, hole_d - 0.001), socket_h)),
            origin=Origin(xyz=(x, y, base_h + socket_h * 0.5)),
            material=stem_mat,
            name=f"switch_socket_{idx}",
        )

    def add_key(
        name: str,
        x: float,
        y: float,
        *,
        width: float = key_w,
        cap_mesh=normal_key_mesh,
        material: Material = key_mat,
        stems: tuple[float, ...] = (0.0,),
    ) -> None:
        key = model.part(name)
        for idx, stem_x in enumerate(stems):
            if len(stems) == 1:
                key.visual(
                    Box((stem_w, stem_d, stem_h)),
                    origin=Origin(xyz=(stem_x, 0.0, stem_center_z)),
                    material=stem_mat,
                    name="stem",
                )
            else:
                key.visual(
                    Box((stem_w, stem_d, stem_h)),
                    origin=Origin(xyz=(stem_x, 0.0, stem_center_z)),
                    material=stem_mat,
                    name=f"stem_{idx}",
                )
        key.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, cap_bottom_z)),
            material=material,
            name="keycap",
        )
        model.articulation(
            f"case_to_{name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, y, PLATE_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=KEY_TRAVEL),
        )

    for row_idx, y in enumerate(row_y[:4]):
        for col_idx, x in enumerate(col_x):
            material = accent_mat if (row_idx == 0 and col_idx == 0) else key_mat
            add_key(f"key_{row_idx}_{col_idx}", x, y, material=material)

    bottom_y = row_y[4]
    for suffix, x in enumerate((col_x[0], col_x[1], col_x[2])):
        add_key(f"modifier_{suffix}", x, bottom_y, material=mod_mat)
    add_key(
        "spacebar",
        0.0,
        bottom_y,
        width=space_key_w,
        cap_mesh=space_key_mesh,
        material=mod_mat,
        stems=(-0.040, 0.0, 0.040),
    )
    for suffix, x in enumerate((col_x[9], col_x[10], col_x[11])):
        add_key(f"modifier_{suffix + 3}", x, bottom_y, material=mod_mat)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    top_left = object_model.get_part("key_0_0")
    home_key = object_model.get_part("key_2_5")
    spacebar = object_model.get_part("spacebar")
    home_slide = object_model.get_articulation("case_to_key_2_5")
    space_slide = object_model.get_articulation("case_to_spacebar")

    ctx.expect_overlap(top_left, case, axes="xy", elem_a="stem", elem_b="top_plate", min_overlap=0.004)
    ctx.expect_overlap(spacebar, case, axes="x", elem_a="keycap", elem_b="top_plate", min_overlap=0.08)
    ctx.expect_gap(home_key, case, axis="z", positive_elem="keycap", negative_elem="top_plate", min_gap=0.004)

    rest_home = ctx.part_world_position(home_key)
    rest_space = ctx.part_world_position(spacebar)
    with ctx.pose({home_slide: KEY_TRAVEL, space_slide: KEY_TRAVEL}):
        pressed_home = ctx.part_world_position(home_key)
        pressed_space = ctx.part_world_position(spacebar)
        ctx.expect_gap(
            home_key,
            case,
            axis="z",
            positive_elem="keycap",
            negative_elem="top_plate",
            min_gap=0.0005,
            name="pressed keycap stays just above plate",
        )

    ctx.check(
        "home key slides down by travel",
        rest_home is not None
        and pressed_home is not None
        and pressed_home[2] < rest_home[2] - KEY_TRAVEL * 0.8,
        details=f"rest={rest_home}, pressed={pressed_home}",
    )
    ctx.check(
        "spacebar slides down by travel",
        rest_space is not None
        and pressed_space is not None
        and pressed_space[2] < rest_space[2] - KEY_TRAVEL * 0.8,
        details=f"rest={rest_space}, pressed={pressed_space}",
    )

    return ctx.report()


object_model = build_object_model()
