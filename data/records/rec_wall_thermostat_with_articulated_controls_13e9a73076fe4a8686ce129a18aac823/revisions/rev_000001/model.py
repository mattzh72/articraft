from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_face,
)


THERMOSTAT_WIDTH = 0.146
THERMOSTAT_HEIGHT = 0.106
THERMOSTAT_DEPTH = 0.031


def _add_front_button(
    model: ArticulatedObject,
    housing,
    button_material,
    *,
    index: int,
    face_pos: tuple[float, float],
) -> None:
    button = model.part(f"button_{index}")
    button.visual(
        Box((0.020, 0.011, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=button_material,
        name="cap",
    )
    button.visual(
        Box((0.016, 0.008, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0011)),
        material=button_material,
        name="plunger",
    )

    model.articulation(
        f"housing_to_button_{index}",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=place_on_face(
            housing,
            "+y",
            face_pos=face_pos,
            proud=0.0,
            prefer_collisions=False,
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0014,
        ),
    )


def _add_side_dial(
    model: ArticulatedObject,
    housing,
    shaft_material,
    dial_material,
    dial_mesh,
    *,
    index: int,
    face_pos: tuple[float, float],
) -> None:
    dial = model.part(f"dial_{index}")
    dial.visual(
        Cylinder(radius=0.0032, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=shaft_material,
        name="shaft",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=dial_material,
        name="cap",
    )

    model.articulation(
        f"housing_to_dial_{index}",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=place_on_face(
            housing,
            "+x",
            face_pos=face_pos,
            proud=0.0,
            prefer_collisions=False,
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="programmable_wall_thermostat")

    shell = model.material("shell", rgba=(0.93, 0.93, 0.91, 1.0))
    trim = model.material("trim", rgba=(0.86, 0.86, 0.84, 1.0))
    panel = model.material("panel", rgba=(0.80, 0.81, 0.79, 1.0))
    button_color = model.material("button", rgba=(0.69, 0.72, 0.74, 1.0))
    display = model.material("display", rgba=(0.15, 0.25, 0.29, 0.85))
    dial_color = model.material("dial", rgba=(0.23, 0.24, 0.25, 1.0))
    shaft = model.material("shaft", rgba=(0.53, 0.54, 0.56, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.126, 0.004, 0.086)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=trim,
        name="back_plate",
    )
    housing.visual(
        Box((THERMOSTAT_WIDTH, 0.025, 0.102)),
        origin=Origin(xyz=(0.0, 0.0155, 0.0)),
        material=shell,
        name="body",
    )
    housing.visual(
        Box((THERMOSTAT_WIDTH, 0.004, THERMOSTAT_HEIGHT)),
        origin=Origin(xyz=(0.0, THERMOSTAT_DEPTH - 0.002, 0.0)),
        material=trim,
        name="face_frame",
    )
    housing.visual(
        Box((0.102, 0.0015, 0.070)),
        origin=Origin(xyz=(-0.006, THERMOSTAT_DEPTH - 0.00075, -0.002)),
        material=panel,
        name="control_panel",
    )
    housing.visual(
        Box((0.060, 0.0012, 0.024)),
        origin=Origin(xyz=(-0.016, THERMOSTAT_DEPTH - 0.0006, 0.018)),
        material=display,
        name="display_window",
    )
    housing.visual(
        Box((0.022, 0.0012, 0.010)),
        origin=Origin(xyz=(0.029, THERMOSTAT_DEPTH - 0.0006, 0.018)),
        material=panel,
        name="status_window",
    )

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.018,
            0.0105,
            body_style="cylindrical",
            edge_radius=0.0010,
            grip=KnobGrip(style="fluted", count=14, depth=0.0007),
            center=False,
        ),
        "thermostat_dial_cap",
    )

    for button_index, x_pos in enumerate((-0.041, -0.013, 0.015, 0.043)):
        _add_front_button(
            model,
            housing,
            button_color,
            index=button_index,
            face_pos=(x_pos, -0.024),
        )

    for dial_index, z_pos in enumerate((0.021, -0.010)):
        _add_side_dial(
            model,
            housing,
            shaft,
            dial_color,
            dial_mesh,
            index=dial_index,
            face_pos=(0.001, z_pos),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"housing_to_button_{index}")
        upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else 0.0

        ctx.expect_gap(
            button,
            housing,
            axis="y",
            max_gap=0.0,
            max_penetration=1e-6,
            name=f"button_{index} sits on the front face",
        )
        ctx.expect_overlap(
            button,
            housing,
            axes="xz",
            min_overlap=0.010,
            name=f"button_{index} stays within the thermostat face",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    for index in range(2):
        dial = object_model.get_part(f"dial_{index}")
        dial_joint = object_model.get_articulation(f"housing_to_dial_{index}")

        ctx.expect_gap(
            dial,
            housing,
            axis="x",
            max_gap=0.0,
            max_penetration=0.0,
            name=f"dial_{index} seats on the right edge",
        )
        ctx.expect_overlap(
            dial,
            housing,
            axes="yz",
            min_overlap=0.008,
            name=f"dial_{index} overlaps the right side footprint",
        )

        rest_pos = ctx.part_world_position(dial)
        with ctx.pose({dial_joint: 1.6}):
            turned_pos = ctx.part_world_position(dial)
        ctx.check(
            f"dial_{index} rotates about a fixed shaft",
            rest_pos is not None
            and turned_pos is not None
            and max(abs(a - b) for a, b in zip(rest_pos, turned_pos)) < 1e-6,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

    return ctx.report()


object_model = build_object_model()
