from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.38
BODY_D = 0.34
BODY_H = 0.78
BODY_BOTTOM_Z = 0.02

GRILLE_W = 0.318
GRILLE_H = 0.56
GRILLE_Y = BODY_D * 0.5 - 0.014
GRILLE_Z = 0.38

DECK_W = 0.19
DECK_D = 0.032
DECK_H = 0.075
DECK_Y = BODY_D * 0.5 + DECK_D * 0.5
DECK_Z = 0.70

SLEEVE_X = 0.11
SLEEVE_Y = -(BODY_D * 0.5) + 0.014
SLEEVE_BOTTOM_Z = 0.52
SLEEVE_LEN = 0.24
SLEEVE_OUTER_R = 0.011
SLEEVE_INNER_R = 0.0085

HANDLE_ROD_R = 0.0075
HANDLE_MEMBER_LEN = 0.60
HANDLE_MEMBER_CENTER_Z = 0.16
HANDLE_GRIP_Z = 0.455
HANDLE_TRAVEL = 0.14

WHEEL_R = 0.06
WHEEL_W = 0.046
WHEEL_Y = -0.145
WHEEL_Z = WHEEL_R
WHEEL_X = BODY_W * 0.5 + 0.024


def _body_shell_mesh():
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    shell = shell.edges("|Z").fillet(0.028)
    shell = shell.faces(">Y").workplane().rect(0.326, 0.592).cutBlind(-0.016)
    shell = shell.faces(">Y").workplane().moveTo(0.0, 0.275).rect(0.206, 0.090).cutBlind(-0.008)
    for x_pos in (-SLEEVE_X, SLEEVE_X):
        shell = shell.faces("<Y").workplane().moveTo(x_pos, 0.16).rect(0.042, 0.48).cutBlind(-0.055)
    return shell


def _sleeve_mesh():
    return (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_R)
        .circle(SLEEVE_INNER_R)
        .extrude(SLEEVE_LEN)
    )


def _deck_mesh():
    pod = cq.Workplane("XY").box(DECK_W, DECK_D, DECK_H)
    pod = pod.edges("|Z").fillet(0.006)
    for x_pos in (-0.056, 0.0, 0.056):
        pod = pod.faces(">Y").workplane().moveTo(x_pos, 0.0).rect(0.022, 0.012).cutBlind(-0.022)
    return pod


def _add_wheel(model: ArticulatedObject, body, name: str, x_pos: float):
    tire_black = "tire_black"
    hub_gray = "hub_gray"

    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.038, length=WHEEL_W * 0.82),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.016, length=WHEEL_W * 1.04),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="axle_cap",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(x_pos, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )


def _add_button(model: ArticulatedObject, deck, index: int, x_pos: float):
    button = model.part(f"button_{index}")
    button.visual(
        Box((0.034, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material="button_black",
        name="cap",
    )
    button.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material="button_core",
        name="stem",
    )
    model.articulation(
        f"deck_to_button_{index}",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=button,
        origin=Origin(xyz=(x_pos, DECK_D * 0.5, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trolley_portable_speaker")

    model.material("body_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("grille_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("wheel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("tire_black", rgba=(0.06, 0.06, 0.06, 1.0))
    model.material("hub_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("handle_metal", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("handle_grip", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("button_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("button_core", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_mesh(), "speaker_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_H * 0.5)),
        material="body_black",
        name="shell",
    )
    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (GRILLE_W, GRILLE_H),
                0.004,
                hole_diameter=0.007,
                pitch=(0.013, 0.013),
                frame=0.012,
                corner_radius=0.018,
                stagger=True,
            ),
            "speaker_grille",
        ),
        origin=Origin(
            xyz=(0.0, GRILLE_Y, GRILLE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="grille_black",
        name="grille",
    )

    sleeve_mesh = mesh_from_cadquery(_sleeve_mesh(), "speaker_handle_sleeve")
    for index, x_pos in enumerate((-SLEEVE_X, SLEEVE_X)):
        body.visual(
            sleeve_mesh,
            origin=Origin(xyz=(x_pos, SLEEVE_Y, SLEEVE_BOTTOM_Z)),
            material="handle_metal",
            name=f"sleeve_{index}",
        )
        body.visual(
            Box((0.026, 0.016, 0.070)),
            origin=Origin(xyz=(x_pos, -(BODY_D * 0.5) - 0.004, SLEEVE_BOTTOM_Z + 0.045)),
            material="trim_black",
            name=f"sleeve_lower_mount_{index}",
        )
        body.visual(
            Box((0.026, 0.016, 0.070)),
            origin=Origin(xyz=(x_pos, -(BODY_D * 0.5) - 0.004, SLEEVE_BOTTOM_Z + SLEEVE_LEN - 0.045)),
            material="trim_black",
            name=f"sleeve_upper_mount_{index}",
        )

    body.visual(
        Box((0.304, 0.016, 0.340)),
        origin=Origin(xyz=(0.0, -(BODY_D * 0.5) - 0.004, 0.640)),
        material="trim_black",
        name="handle_backplate",
    )

    for index, x_pos in enumerate((-WHEEL_X, WHEEL_X)):
        body.visual(
            Box((0.020, 0.024, 0.090)),
            origin=Origin(xyz=(x_pos * 0.76, WHEEL_Y - 0.012, 0.090)),
            material="trim_black",
            name=f"wheel_bracket_{index}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(
                xyz=(x_pos - math.copysign(WHEEL_W * 0.52 + 0.006, x_pos), WHEEL_Y, WHEEL_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="hub_gray",
            name=f"wheel_boss_{index}",
        )

    body.visual(
        Box((0.042, 0.030, 0.016)),
        origin=Origin(xyz=(-0.115, 0.120, 0.014)),
        material="trim_black",
        name="foot_0",
    )
    body.visual(
        Box((0.042, 0.030, 0.016)),
        origin=Origin(xyz=(0.115, 0.120, 0.014)),
        material="trim_black",
        name="foot_1",
    )

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_deck_mesh(), "speaker_control_deck"),
        material="trim_black",
        name="pod",
    )
    model.articulation(
        "body_to_deck",
        ArticulationType.FIXED,
        parent=body,
        child=deck,
        origin=Origin(xyz=(0.0, DECK_Y, DECK_Z)),
    )

    _add_button(model, deck, 0, -0.056)
    _add_button(model, deck, 1, 0.0)
    _add_button(model, deck, 2, 0.056)

    handle = model.part("handle")
    for index, x_pos in enumerate((-SLEEVE_X, SLEEVE_X)):
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_MEMBER_LEN),
            origin=Origin(xyz=(x_pos, SLEEVE_Y, HANDLE_MEMBER_CENTER_Z)),
            material="handle_metal",
            name=f"rod_{index}",
        )
        handle.visual(
            Cylinder(radius=0.0135, length=0.012),
            origin=Origin(xyz=(x_pos, SLEEVE_Y, SLEEVE_LEN + 0.006)),
            material="handle_grip",
            name=f"stop_{index}",
        )
    handle.visual(
        Cylinder(radius=0.012, length=0.22),
        origin=Origin(
            xyz=(0.0, SLEEVE_Y, HANDLE_GRIP_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="handle_metal",
        name="crossbar",
    )
    handle.visual(
        Box((0.12, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, SLEEVE_Y, HANDLE_GRIP_Z)),
        material="handle_grip",
        name="grip",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )

    _add_wheel(model, body, "left_wheel", -WHEEL_X)
    _add_wheel(model, body, "right_wheel", WHEEL_X)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="rod_0",
        outer_elem="sleeve_0",
        margin=0.004,
        name="left handle rod stays inside left sleeve footprint",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="rod_1",
        outer_elem="sleeve_1",
        margin=0.004,
        name="right handle rod stays inside right sleeve footprint",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="rod_0",
        elem_b="sleeve_0",
        min_overlap=0.22,
        name="collapsed left handle rod remains inserted",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="rod_1",
        elem_b="sleeve_1",
        min_overlap=0.22,
        name="collapsed right handle rod remains inserted",
    )

    handle_rest = ctx.part_world_position(handle)
    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: handle_limits.upper}):
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="rod_0",
                elem_b="sleeve_0",
                min_overlap=0.10,
                name="extended left handle rod still remains inserted",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="rod_1",
                elem_b="sleeve_1",
                min_overlap=0.10,
                name="extended right handle rod still remains inserted",
            )
            handle_extended = ctx.part_world_position(handle)
        ctx.check(
            "pull handle extends upward",
            handle_rest is not None
            and handle_extended is not None
            and handle_extended[2] > handle_rest[2] + 0.10,
            details=f"rest={handle_rest}, extended={handle_extended}",
        )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"deck_to_button_{index}")
        ctx.expect_overlap(
            button,
            deck,
            axes="xz",
            min_overlap=0.018,
            name=f"button {index} stays registered to the deck face",
        )
        button_rest = ctx.part_world_position(button)
        limits = button_joint.motion_limits
        if limits is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                button_pressed = ctx.part_world_position(button)
            ctx.check(
                f"button {index} depresses inward",
                button_rest is not None
                and button_pressed is not None
                and button_pressed[1] < button_rest[1] - 0.002,
                details=f"rest={button_rest}, pressed={button_pressed}",
            )

    left_pos = ctx.part_world_position(left_wheel)
    right_pos = ctx.part_world_position(right_wheel)
    ctx.check(
        "transport wheels sit at the rear lower corners",
        left_pos is not None
        and right_pos is not None
        and left_pos[0] < -0.20
        and right_pos[0] > 0.20
        and left_pos[1] < -0.10
        and right_pos[1] < -0.10
        and abs(left_pos[2] - WHEEL_R) < 0.01
        and abs(right_pos[2] - WHEEL_R) < 0.01,
        details=f"left={left_pos}, right={right_pos}",
    )

    return ctx.report()


object_model = build_object_model()
