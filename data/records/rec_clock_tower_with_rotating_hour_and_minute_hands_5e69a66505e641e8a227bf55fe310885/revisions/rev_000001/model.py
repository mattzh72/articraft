from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clock_tower")

    tower = model.part("tower")
    tower.visual(
        Box((3.0, 3.0, 15.0)),
        origin=Origin(xyz=(0.0, 0.0, 7.5)),
        name="tower_body",
        material=Material(color=(0.2, 0.2, 0.2), name="steel"),
    )

    directions = [
        ("front", (0.0, 1.0, 0.0), (0.0, 1.5, 12.0), (-1.57079632679, 0.0, 0.0)),
        ("back", (0.0, -1.0, 0.0), (0.0, -1.5, 12.0), (1.57079632679, 0.0, 0.0)),
        ("right", (1.0, 0.0, 0.0), (1.5, 0.0, 12.0), (0.0, 1.57079632679, 0.0)),
        ("left", (-1.0, 0.0, 0.0), (-1.5, 0.0, 12.0), (0.0, -1.57079632679, 0.0)),
    ]

    for name, normal, center, rpy in directions:
        nx, ny, nz = normal
        cx, cy, cz = center

        # Clock face
        face_pos = (cx + nx * 0.05, cy + ny * 0.05, cz + nz * 0.05)
        tower.visual(
            Cylinder(radius=1.2, height=0.1),
            origin=Origin(xyz=face_pos, rpy=rpy),
            name=f"face_{name}",
            material=Material(color=(0.9, 0.9, 0.9), name="glass"),
        )

        # Central pin
        pin_pos = (cx + nx * 0.09, cy + ny * 0.09, cz + nz * 0.09)
        tower.visual(
            Cylinder(radius=0.08, height=0.18),
            origin=Origin(xyz=pin_pos, rpy=rpy),
            name=f"pin_{name}",
            material=Material(color=(0.1, 0.1, 0.1), name="black_metal"),
        )

        # Hour hand
        hour_hand = model.part(f"hour_hand_{name}")
        if nx != 0:
            # X normal
            hour_vis = Box((0.02, 0.1, 0.8))
            hour_axis = (nx, 0.0, 0.0)
        else:
            # Y normal
            hour_vis = Box((0.1, 0.02, 0.8))
            hour_axis = (0.0, ny, 0.0)

        hour_hand.visual(
            hour_vis,
            origin=Origin(xyz=(0.0, 0.0, 0.3)),
            name=f"hour_hand_vis_{name}",
            material=Material(color=(0.1, 0.1, 0.1), name="black_metal"),
        )

        hour_orig = Origin(xyz=(cx + nx * 0.12, cy + ny * 0.12, cz + nz * 0.12))
        model.articulation(
            f"hour_joint_{name}",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour_hand,
            origin=hour_orig,
            axis=hour_axis,
            motion_limits=MotionLimits(effort=1.0, velocity=0.1),
        )

        # Minute hand
        minute_hand = model.part(f"minute_hand_{name}")
        if nx != 0:
            # X normal
            min_vis = Box((0.02, 0.06, 1.2))
            min_axis = (nx, 0.0, 0.0)
        else:
            # Y normal
            min_vis = Box((0.06, 0.02, 1.2))
            min_axis = (0.0, ny, 0.0)

        minute_hand.visual(
            min_vis,
            origin=Origin(xyz=(0.0, 0.0, 0.5)),
            name=f"minute_hand_vis_{name}",
            material=Material(color=(0.1, 0.1, 0.1), name="black_metal"),
        )

        min_orig = Origin(xyz=(cx + nx * 0.16, cy + ny * 0.16, cz + nz * 0.16))
        model.articulation(
            f"minute_joint_{name}",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=minute_hand,
            origin=min_orig,
            axis=min_axis,
            motion_limits=MotionLimits(effort=1.0, velocity=1.2),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")

    for name in ["front", "back", "right", "left"]:
        hour_hand = object_model.get_part(f"hour_hand_{name}")
        min_hand = object_model.get_part(f"minute_hand_{name}")

        # The hands intentionally overlap the central pin on the tower
        ctx.allow_overlap(
            hour_hand,
            tower,
            elem_a=f"hour_hand_vis_{name}",
            elem_b=f"pin_{name}",
            reason="The hour hand is mounted on the central pin.",
        )
        ctx.allow_overlap(
            min_hand,
            tower,
            elem_a=f"minute_hand_vis_{name}",
            elem_b=f"pin_{name}",
            reason="The minute hand is mounted on the central pin.",
        )

        # The hands should not overlap each other
        ctx.expect_contact(
            min_hand,
            hour_hand,
            contact_tol=0.03,
            name=f"{name} hands are close but do not overlap",
        )

    return ctx.report()


object_model = build_object_model()
