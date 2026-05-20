from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


TRAY_WIDTH = 0.66
TRAY_DEPTH = 0.28
TRAY_FLOOR_THICKNESS = 0.003
TRAY_WALL_THICKNESS = 0.008
TRAY_WALL_HEIGHT = 0.040

BRACKET_WIDTH = 0.188
BRACKET_DEPTH = 0.115
BRACKET_BASE_THICKNESS = 0.004
BRACKET_SIDE_THICKNESS = 0.007
BRACKET_SIDE_HEIGHT = 0.050
HINGE_Z = 0.054
HINGE_Y = 0.054

BODY_WIDTH = 0.160
BODY_DEPTH = 0.118
BODY_HEIGHT = 0.047


def _pedal_section(y: float, width: float, bottom_z: float, top_z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    edge_bevel = 0.008
    corner_drop = 0.006
    top_chamfer = 0.004
    return [
        (-half_w + edge_bevel, y, bottom_z),
        (half_w - edge_bevel, y, bottom_z),
        (half_w, y, bottom_z + corner_drop),
        (half_w, y, top_z - top_chamfer),
        (half_w - edge_bevel, y, top_z),
        (-half_w + edge_bevel, y, top_z),
        (-half_w, y, top_z - top_chamfer),
        (-half_w, y, bottom_z + corner_drop),
    ]


def _build_pedal_shell_mesh():
    shell_geom = section_loft(
        [
            _pedal_section(0.000, BODY_WIDTH, -0.043, 0.001),
            _pedal_section(-0.035, BODY_WIDTH, -0.044, -0.002),
            _pedal_section(-0.078, BODY_WIDTH * 0.9875, -0.045, -0.008),
            _pedal_section(-BODY_DEPTH, BODY_WIDTH * 0.9625, -0.046, -0.013),
        ]
    )
    return mesh_from_geometry(shell_geom, "stompbox_body_shell")


def _add_bracket_geometry(part, *, bracket_material) -> None:
    part.visual(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, BRACKET_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_BASE_THICKNESS * 0.5)),
        material=bracket_material,
        name="bracket_base",
    )
    part.visual(
        Box((BRACKET_SIDE_THICKNESS, BRACKET_DEPTH - 0.010, BRACKET_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BRACKET_WIDTH * 0.5 - BRACKET_SIDE_THICKNESS * 0.5),
                -0.002,
                BRACKET_SIDE_HEIGHT * 0.5,
            )
        ),
        material=bracket_material,
        name="left_cheek",
    )
    part.visual(
        Box((BRACKET_SIDE_THICKNESS, BRACKET_DEPTH - 0.010, BRACKET_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                BRACKET_WIDTH * 0.5 - BRACKET_SIDE_THICKNESS * 0.5,
                -0.002,
                BRACKET_SIDE_HEIGHT * 0.5,
            )
        ),
        material=bracket_material,
        name="right_cheek",
    )
    part.visual(
        Box((BRACKET_WIDTH - 0.022, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -(BRACKET_DEPTH * 0.5 - 0.004), 0.003)),
        material=bracket_material,
        name="front_lip",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(
            xyz=(-0.088, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bracket_material,
        name="left_hinge_barrel",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(
            xyz=(0.088, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bracket_material,
        name="right_hinge_barrel",
    )
    part.inertial = Inertial.from_geometry(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, 0.060)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )


def _add_body_geometry(part, *, shell_mesh, shell_material, hardware_material, accent_material) -> None:
    part.visual(
        shell_mesh,
        material=shell_material,
        name="body_shell",
    )
    part.visual(
        Cylinder(radius=0.0065, length=0.164),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_material,
        name="body_hinge_barrel",
    )
    part.visual(
        Box((0.112, 0.042, 0.0015)),
        origin=Origin(xyz=(0.0, -0.034, -0.0015)),
        material=accent_material,
        name="control_strip",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, -0.085, 0.0)),
        material=hardware_material,
        name="footswitch",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(0.043, -0.068, -0.004)),
        material=accent_material,
        name="status_led",
    )
    part.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.72,
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5, -0.022)),
    )


def _add_knob_geometry(part, *, knob_material, marker_material) -> None:
    part.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=knob_material,
        name="knob_body",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=knob_material,
        name="knob_cap",
    )
    part.visual(
        Box((0.003, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.008, 0.0125)),
        material=marker_material,
        name="pointer_mark",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.014),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_effects_chain_stomp_box_board")

    tray_black = model.material("tray_black", rgba=(0.10, 0.11, 0.12, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    knob_black = model.material("knob_black", rgba=(0.07, 0.07, 0.08, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.83, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    panel_black = model.material("panel_black", rgba=(0.13, 0.13, 0.14, 1.0))
    violet_paint = model.material("violet_paint", rgba=(0.49, 0.25, 0.74, 1.0))
    teal_paint = model.material("teal_paint", rgba=(0.14, 0.56, 0.55, 1.0))
    amber_paint = model.material("amber_paint", rgba=(0.73, 0.46, 0.12, 1.0))
    led_red = model.material("led_red", rgba=(0.85, 0.18, 0.15, 1.0))
    led_green = model.material("led_green", rgba=(0.20, 0.80, 0.34, 1.0))
    led_blue = model.material("led_blue", rgba=(0.24, 0.48, 0.92, 1.0))

    shell_mesh = _build_pedal_shell_mesh()

    tray_base = model.part("tray_base")
    tray_base.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_FLOOR_THICKNESS * 0.5)),
        material=tray_black,
        name="tray_floor",
    )
    tray_base.visual(
        Box((TRAY_WIDTH, TRAY_WALL_THICKNESS, TRAY_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(TRAY_DEPTH * 0.5 - TRAY_WALL_THICKNESS * 0.5),
                TRAY_WALL_HEIGHT * 0.5 + 0.001,
            )
        ),
        material=tray_black,
        name="front_rail",
    )
    tray_base.visual(
        Box((TRAY_WIDTH, TRAY_WALL_THICKNESS, TRAY_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                TRAY_DEPTH * 0.5 - TRAY_WALL_THICKNESS * 0.5,
                TRAY_WALL_HEIGHT * 0.5 + 0.001,
            )
        ),
        material=tray_black,
        name="rear_rail",
    )
    tray_base.visual(
        Box((TRAY_WALL_THICKNESS, TRAY_DEPTH - 0.016, TRAY_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(TRAY_WIDTH * 0.5 - TRAY_WALL_THICKNESS * 0.5),
                0.0,
                TRAY_WALL_HEIGHT * 0.5 + 0.001,
            )
        ),
        material=tray_black,
        name="left_rail",
    )
    tray_base.visual(
        Box((TRAY_WALL_THICKNESS, TRAY_DEPTH - 0.016, TRAY_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                TRAY_WIDTH * 0.5 - TRAY_WALL_THICKNESS * 0.5,
                0.0,
                TRAY_WALL_HEIGHT * 0.5 + 0.001,
            )
        ),
        material=tray_black,
        name="right_rail",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        [
            (-0.270, -0.102),
            (0.270, -0.102),
            (-0.270, 0.102),
            (0.270, 0.102),
        ],
        start=1,
    ):
        tray_base.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(foot_x, foot_y, -0.0055)),
            material=rubber,
            name=f"foot_{foot_index}",
        )
    tray_base.inertial = Inertial.from_geometry(
        Box((TRAY_WIDTH, TRAY_DEPTH, 0.055)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    unit_specs = [
        {
            "prefix": "delay",
            "x": -0.195,
            "body_material": violet_paint,
            "led_material": led_red,
        },
        {
            "prefix": "filter",
            "x": 0.0,
            "body_material": teal_paint,
            "led_material": led_green,
        },
        {
            "prefix": "looper",
            "x": 0.195,
            "body_material": amber_paint,
            "led_material": led_blue,
        },
    ]

    for spec in unit_specs:
        prefix = spec["prefix"]

        bracket = model.part(f"{prefix}_bracket")
        _add_bracket_geometry(bracket, bracket_material=bracket_steel)
        model.articulation(
            f"tray_to_{prefix}_bracket",
            ArticulationType.FIXED,
            parent=tray_base,
            child=bracket,
            origin=Origin(xyz=(spec["x"], 0.0, TRAY_FLOOR_THICKNESS)),
        )

        body = model.part(f"{prefix}_body")
        _add_body_geometry(
            body,
            shell_mesh=shell_mesh,
            shell_material=spec["body_material"],
            hardware_material=stainless,
            accent_material=spec["led_material"],
        )
        model.articulation(
            f"{prefix}_hinge",
            ArticulationType.REVOLUTE,
            parent=bracket,
            child=body,
            origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.5,
                lower=0.0,
                upper=1.10,
            ),
        )

        knob_mounts = [
            (-0.046, -0.034, -0.0020),
            (0.0, -0.032, -0.0018),
            (0.046, -0.034, -0.0020),
        ]
        for knob_index, (knob_x, knob_y, knob_z) in enumerate(knob_mounts, start=1):
            knob = model.part(f"{prefix}_knob_{knob_index}")
            _add_knob_geometry(knob, knob_material=knob_black, marker_material=panel_black)
            model.articulation(
                f"{prefix}_knob_{knob_index}_spin",
                ArticulationType.CONTINUOUS,
                parent=body,
                child=knob,
                origin=Origin(xyz=(knob_x, knob_y, knob_z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=0.2, velocity=10.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    tray = object_model.get_part("tray_base")
    unit_prefixes = ("delay", "filter", "looper")

    for prefix in unit_prefixes:
        bracket = object_model.get_part(f"{prefix}_bracket")
        body = object_model.get_part(f"{prefix}_body")
        hinge = object_model.get_articulation(f"{prefix}_hinge")

        ctx.expect_contact(
            bracket,
            tray,
            elem_a="bracket_base",
            elem_b="tray_floor",
            name=f"{prefix} bracket sits on the tray floor",
        )

        with ctx.pose({hinge: 0.0}):
            ctx.expect_overlap(
                body,
                bracket,
                axes="xy",
                min_overlap=0.10,
                name=f"{prefix} body stays nested over its bracket at rest",
            )
            closed_aabb = ctx.part_world_aabb(body)

        upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None
        with ctx.pose({hinge: upper or 0.0}):
            open_aabb = ctx.part_world_aabb(body)

        ctx.check(
            f"{prefix} body lifts for battery access",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.050,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

        for knob_index in (1, 2, 3):
            knob = object_model.get_part(f"{prefix}_knob_{knob_index}")
            knob_joint = object_model.get_articulation(f"{prefix}_knob_{knob_index}_spin")
            joint_kind = getattr(knob_joint.articulation_type, "name", str(knob_joint.articulation_type))
            limits = knob_joint.motion_limits

            ctx.expect_overlap(
                knob,
                body,
                axes="xy",
                min_overlap=0.015,
                name=f"{prefix} knob {knob_index} sits on the control face",
            )
            ctx.check(
                f"{prefix} knob {knob_index} rotates continuously",
                "CONTINUOUS" in str(joint_kind).upper()
                and limits is not None
                and limits.lower is None
                and limits.upper is None
                and knob_joint.axis == (0.0, 0.0, 1.0),
                details=f"joint_type={joint_kind}, axis={knob_joint.axis}, limits={limits}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
