from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.320
CABINET_DEPTH = 0.560
CABINET_HEIGHT = 0.620
PANEL_THICKNESS = 0.016
BACK_THICKNESS = 0.012
FRONT_THICKNESS = 0.018
DRAWER_BODY_WIDTH = 0.270
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.012
DRAWER_BACK_THICKNESS = 0.012
RUNNER_THICKNESS = 0.009
RUNNER_HEIGHT = 0.018
RUNNER_LENGTH = 0.440
RUNNER_FRONT_SETBACK = 0.045

DRAWER_SPECS = (
    {
        "name": "top_drawer",
        "front_height": 0.105,
        "body_height": 0.072,
        "bottom_z": 0.495,
        "travel": 0.320,
        "runner_z": 0.040,
        "has_lock": True,
        "file_drawer": False,
    },
    {
        "name": "upper_drawer",
        "front_height": 0.105,
        "body_height": 0.072,
        "bottom_z": 0.385,
        "travel": 0.320,
        "runner_z": 0.040,
        "has_lock": False,
        "file_drawer": False,
    },
    {
        "name": "lower_drawer",
        "front_height": 0.105,
        "body_height": 0.072,
        "bottom_z": 0.275,
        "travel": 0.320,
        "runner_z": 0.040,
        "has_lock": False,
        "file_drawer": False,
    },
    {
        "name": "file_drawer",
        "front_height": 0.250,
        "body_height": 0.192,
        "bottom_z": 0.020,
        "travel": 0.340,
        "runner_z": 0.082,
        "has_lock": False,
        "file_drawer": True,
    },
)


def _add_box(part, size, xyz, material, *, name: str | None = None) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz,
    rpy=(0.0, 0.0, 0.0),
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_handle(part, *, front_width: float, front_height: float) -> None:
    handle_span = min(front_width - 0.140, 0.170)
    stem_offset = handle_span * 0.44
    handle_z = front_height * 0.56

    _add_cylinder(
        part,
        radius=0.0045,
        length=0.020,
        xyz=(-stem_offset, 0.010, handle_z),
        rpy=(-pi / 2.0, 0.0, 0.0),
        material="pull_metal",
        name="pull_stem_left",
    )
    _add_cylinder(
        part,
        radius=0.0045,
        length=0.020,
        xyz=(stem_offset, 0.010, handle_z),
        rpy=(-pi / 2.0, 0.0, 0.0),
        material="pull_metal",
        name="pull_stem_right",
    )
    _add_cylinder(
        part,
        radius=0.0055,
        length=handle_span,
        xyz=(0.0, 0.022, handle_z),
        rpy=(0.0, pi / 2.0, 0.0),
        material="pull_metal",
        name="pull_bar",
    )


def _add_carcass(model: ArticulatedObject):
    carcass = model.part("carcass")
    inner_width = CABINET_WIDTH - (2.0 * PANEL_THICKNESS)

    _add_box(
        carcass,
        (PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT),
        (
            -(CABINET_WIDTH / 2.0) + (PANEL_THICKNESS / 2.0),
            -(CABINET_DEPTH / 2.0),
            CABINET_HEIGHT / 2.0,
        ),
        "body_paint",
        name="left_side",
    )
    _add_box(
        carcass,
        (PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT),
        (
            (CABINET_WIDTH / 2.0) - (PANEL_THICKNESS / 2.0),
            -(CABINET_DEPTH / 2.0),
            CABINET_HEIGHT / 2.0,
        ),
        "body_paint",
        name="right_side",
    )
    _add_box(
        carcass,
        (inner_width, CABINET_DEPTH, PANEL_THICKNESS),
        (0.0, -(CABINET_DEPTH / 2.0), CABINET_HEIGHT - (PANEL_THICKNESS / 2.0)),
        "body_paint",
        name="top_panel",
    )
    _add_box(
        carcass,
        (inner_width, CABINET_DEPTH, PANEL_THICKNESS),
        (0.0, -(CABINET_DEPTH / 2.0), PANEL_THICKNESS / 2.0),
        "body_paint",
        name="bottom_panel",
    )
    _add_box(
        carcass,
        (inner_width, BACK_THICKNESS, CABINET_HEIGHT - (2.0 * PANEL_THICKNESS)),
        (
            0.0,
            -CABINET_DEPTH + (BACK_THICKNESS / 2.0),
            CABINET_HEIGHT / 2.0,
        ),
        "body_paint_dark",
        name="back_panel",
    )

    skid_length = CABINET_DEPTH - 0.090
    skid_width = 0.028
    skid_height = 0.014
    skid_z = -(skid_height / 2.0)
    skid_y = -(CABINET_DEPTH / 2.0) - 0.010
    for index, x_pos in enumerate((-0.082, 0.082)):
        _add_box(
            carcass,
            (skid_width, skid_length, skid_height),
            (x_pos, skid_y, skid_z),
            "base_trim",
            name=f"skid_{index}",
        )

    runner_x = (DRAWER_BODY_WIDTH / 2.0) + (RUNNER_THICKNESS / 2.0)
    runner_y = -(RUNNER_FRONT_SETBACK + (RUNNER_LENGTH / 2.0))
    top_runner_z = DRAWER_SPECS[0]["bottom_z"] + DRAWER_SPECS[0]["runner_z"]
    upper_runner_z = DRAWER_SPECS[1]["bottom_z"] + DRAWER_SPECS[1]["runner_z"]
    lower_runner_z = DRAWER_SPECS[2]["bottom_z"] + DRAWER_SPECS[2]["runner_z"]
    file_runner_z = DRAWER_SPECS[3]["bottom_z"] + DRAWER_SPECS[3]["runner_z"]

    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (-runner_x, runner_y, top_runner_z),
        "runner_metal",
        name="top_drawer_runner_left",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (runner_x, runner_y, top_runner_z),
        "runner_metal",
        name="top_drawer_runner_right",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (-runner_x, runner_y, upper_runner_z),
        "runner_metal",
        name="upper_drawer_runner_left",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (runner_x, runner_y, upper_runner_z),
        "runner_metal",
        name="upper_drawer_runner_right",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (-runner_x, runner_y, lower_runner_z),
        "runner_metal",
        name="lower_drawer_runner_left",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (runner_x, runner_y, lower_runner_z),
        "runner_metal",
        name="lower_drawer_runner_right",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (-runner_x, runner_y, file_runner_z),
        "runner_metal",
        name="file_drawer_runner_left",
    )
    _add_box(
        carcass,
        (RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT),
        (runner_x, runner_y, file_runner_z),
        "runner_metal",
        name="file_drawer_runner_right",
    )

    return carcass


def _add_drawer(model: ArticulatedObject, carcass, spec: dict[str, object]) -> None:
    drawer = model.part(spec["name"])
    front_height = float(spec["front_height"])
    body_height = float(spec["body_height"])
    runner_z = float(spec["runner_z"])
    travel = float(spec["travel"])

    drawer_front_width = CABINET_WIDTH - (2.0 * PANEL_THICKNESS) - 0.004
    side_depth = 0.434
    back_y = -(FRONT_THICKNESS + side_depth + (DRAWER_BACK_THICKNESS / 2.0))
    side_y = -(FRONT_THICKNESS + (side_depth / 2.0))
    body_y = side_y

    _add_box(
        drawer,
        (drawer_front_width, FRONT_THICKNESS, front_height),
        (0.0, -(FRONT_THICKNESS / 2.0), front_height / 2.0),
        "drawer_front",
        name="front",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_THICKNESS, side_depth, body_height),
        (
            -(DRAWER_BODY_WIDTH / 2.0) + (DRAWER_SIDE_THICKNESS / 2.0),
            side_y,
            DRAWER_BOTTOM_THICKNESS + (body_height / 2.0),
        ),
        "drawer_box",
        name="side_left",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_THICKNESS, side_depth, body_height),
        (
            (DRAWER_BODY_WIDTH / 2.0) - (DRAWER_SIDE_THICKNESS / 2.0),
            side_y,
            DRAWER_BOTTOM_THICKNESS + (body_height / 2.0),
        ),
        "drawer_box",
        name="side_right",
    )
    _add_box(
        drawer,
        (
            DRAWER_BODY_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
            side_depth,
            DRAWER_BOTTOM_THICKNESS,
        ),
        (0.0, body_y, DRAWER_BOTTOM_THICKNESS / 2.0),
        "drawer_box",
        name="bottom",
    )
    _add_box(
        drawer,
        (
            DRAWER_BODY_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
            DRAWER_BACK_THICKNESS,
            body_height,
        ),
        (
            0.0,
            back_y,
            DRAWER_BOTTOM_THICKNESS + (body_height / 2.0),
        ),
        "drawer_box",
        name="back",
    )

    _add_handle(drawer, front_width=drawer_front_width, front_height=front_height)

    if bool(spec["has_lock"]):
        _add_cylinder(
            drawer,
            radius=0.008,
            length=0.012,
            xyz=((drawer_front_width / 2.0) - 0.038, 0.006, front_height - 0.026),
            rpy=(-pi / 2.0, 0.0, 0.0),
            material="pull_metal",
            name="lock_core",
        )

    if bool(spec["file_drawer"]):
        file_rail_width = 0.016
        file_rail_height = 0.010
        file_rail_length = side_depth
        file_rail_z = DRAWER_BOTTOM_THICKNESS + body_height - (file_rail_height / 2.0) - 0.010
        file_rail_x = (
            (DRAWER_BODY_WIDTH / 2.0)
            - DRAWER_SIDE_THICKNESS
            - (file_rail_width / 2.0)
        )
        for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
            _add_box(
                drawer,
                (file_rail_width, file_rail_length, file_rail_height),
                (x_sign * file_rail_x, side_y, file_rail_z),
                "runner_metal",
                name=f"file_rail_{side_name}",
            )

    model.articulation(
        f"carcass_to_{spec['name']}",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, float(spec["bottom_z"]))),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=travel,
            effort=140.0 if bool(spec["file_drawer"]) else 90.0,
            velocity=0.45,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pedestal_cabinet")

    model.material("body_paint", rgba=(0.84, 0.85, 0.86, 1.0))
    model.material("body_paint_dark", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("drawer_front", rgba=(0.91, 0.92, 0.93, 1.0))
    model.material("drawer_box", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("runner_metal", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("pull_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("base_trim", rgba=(0.16, 0.17, 0.18, 1.0))

    carcass = _add_carcass(model)
    for spec in DRAWER_SPECS:
        _add_drawer(model, carcass, spec)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    top_drawer = object_model.get_part("top_drawer")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    file_drawer = object_model.get_part("file_drawer")

    top_joint = object_model.get_articulation("carcass_to_top_drawer")
    file_joint = object_model.get_articulation("carcass_to_file_drawer")

    ctx.expect_contact(
        top_drawer,
        carcass,
        elem_a="side_left",
        elem_b="top_drawer_runner_left",
        name="top drawer left side is carried by its runner when closed",
    )
    ctx.expect_overlap(
        top_drawer,
        carcass,
        axes="y",
        elem_a="side_left",
        elem_b="top_drawer_runner_left",
        min_overlap=0.40,
        name="top drawer remains deeply inserted when closed",
    )
    ctx.expect_contact(
        file_drawer,
        carcass,
        elem_a="side_left",
        elem_b="file_drawer_runner_left",
        name="file drawer left side is carried by its runner when closed",
    )
    ctx.expect_overlap(
        file_drawer,
        carcass,
        axes="y",
        elem_a="side_left",
        elem_b="file_drawer_runner_left",
        min_overlap=0.40,
        name="file drawer remains deeply inserted when closed",
    )

    rest_top_pos = ctx.part_world_position(top_drawer)
    rest_file_pos = ctx.part_world_position(file_drawer)

    with ctx.pose(
        {
            top_joint: top_joint.motion_limits.upper,
            file_joint: file_joint.motion_limits.upper,
        }
    ):
        ctx.expect_contact(
            top_drawer,
            carcass,
            elem_a="side_left",
            elem_b="top_drawer_runner_left",
            name="top drawer keeps runner contact at full extension",
        )
        ctx.expect_overlap(
            top_drawer,
            carcass,
            axes="y",
            elem_a="side_left",
            elem_b="top_drawer_runner_left",
            min_overlap=0.08,
            name="top drawer runner still retains insertion at full extension",
        )
        ctx.expect_contact(
            file_drawer,
            carcass,
            elem_a="side_left",
            elem_b="file_drawer_runner_left",
            name="file drawer keeps runner contact at full extension",
        )
        ctx.expect_overlap(
            file_drawer,
            carcass,
            axes="y",
            elem_a="side_left",
            elem_b="file_drawer_runner_left",
            min_overlap=0.06,
            name="file drawer runner still retains insertion at full extension",
        )
        extended_top_pos = ctx.part_world_position(top_drawer)
        extended_file_pos = ctx.part_world_position(file_drawer)

    ctx.check(
        "top drawer extends forward along +Y",
        rest_top_pos is not None
        and extended_top_pos is not None
        and extended_top_pos[1] > rest_top_pos[1] + 0.25,
        details=f"rest={rest_top_pos}, extended={extended_top_pos}",
    )
    ctx.check(
        "file drawer extends forward along +Y",
        rest_file_pos is not None
        and extended_file_pos is not None
        and extended_file_pos[1] > rest_file_pos[1] + 0.28,
        details=f"rest={rest_file_pos}, extended={extended_file_pos}",
    )

    def front_height(part_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="front")
        if aabb is None:
            return None
        return aabb[1][2] - aabb[0][2]

    utility_heights = [
        front_height("top_drawer"),
        front_height("upper_drawer"),
        front_height("lower_drawer"),
    ]
    file_height = front_height("file_drawer")
    utility_ok = all(height is not None for height in utility_heights)
    utility_values = [height for height in utility_heights if height is not None]

    ctx.check(
        "three utility drawer faces share the same height",
        utility_ok
        and max(utility_values) - min(utility_values) < 0.002,
        details=f"utility_heights={utility_heights}",
    )
    ctx.check(
        "bottom file drawer face is visibly taller than the utility drawers",
        file_height is not None
        and utility_values
        and file_height > utility_values[0] + 0.12,
        details=f"utility_heights={utility_heights}, file_height={file_height}",
    )

    return ctx.report()


object_model = build_object_model()
