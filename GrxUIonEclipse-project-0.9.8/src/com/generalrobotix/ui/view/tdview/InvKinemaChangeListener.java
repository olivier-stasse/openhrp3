/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * General Robotix Inc.
 * National Institute of Advanced Industrial Science and Technology (AIST) 
 */
/**
 * InvKinemaChangeListener.java
 *
 *   TransformGroup の変更を BehaviorManager に通知する
 *   ために使う Listener Interface
 *
 * @author  Kernel Co.,Ltd.
 * @version 1.0 (2001/3/1)
 */
package com.generalrobotix.ui.view.tdview;

import java.util.EventListener;

public interface InvKinemaChangeListener extends EventListener
{
    /**
     * コンストラクタ
     *
     * @param   event    イベント
     */
    public void invKinemaChanged(TransformChangeEvent event);
}
